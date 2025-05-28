import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
import math
import time
import pyzed.sl as sl  # 导入 ZED SDK
import numpy as np

class GoToPoseTopicNode(Node):
    def __init__(self):
        super().__init__('move_pos_topic_node')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/Odometry', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(Point, '/goal_pose', self.goal_callback, 10)
        self.lidar_subscription = self.create_subscription(
            Float32MultiArray,
            'horizontal_distances',
            self.lidar_callback,
            10
        )
        self.current_pose = None
        self.target_point = None
        self.has_goal = False
        self.front_zed_far = 0.0
        self.front_zed_near = 0.0
        self.left_zed = 0.0
        self.right_zed = 0.0
        self.front_avg = 0.0
        self.left_distance = 0.0
        self.right_distance = 0.0
        self.front_distance = 0.0
        self.latest_distances = None
        self.current_speed = 0.0
        self.zed = sl.Camera()
        self.init_zed_camera()
        self.avoid_obstacle = False
        self.avoid_obstacle_count = 0

    def init_zed_camera(self):
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.depth_mode = sl.DEPTH_MODE.ULTRA
        init_params.coordinate_units = sl.UNIT.METER
        status = self.zed.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f"无法打开 ZED 相机: {status}")
            exit(1)
        self.depth = sl.Mat()

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        # self.get_logger().info(f"收到里程计: x={self.current_pose.position.x:.3f}, y={self.current_pose.position.y:.3f}")

    def goal_callback(self, msg):
        self.target_point = msg
        self.has_goal = True
        # self.get_logger().info(f"收到新目标点: x={msg.x:.3f}, y={msg.y:.3f}")

    def lidar_callback(self, msg):
        self.latest_distances = msg.data
        num_points = len(self.latest_distances)
        if num_points != 21:
            self.get_logger().warn("接收到的水平距离数据数量与预期不符！")
            return
        front_distances = self.latest_distances[8:13]
        left_distances = self.latest_distances[13:]
        right_distances = self.latest_distances[:8]
        self.front_distance = min(front_distances)
        self.left_distance = sum(left_distances) / len(left_distances)
        self.right_distance = sum(right_distances) / len(right_distances)

    def get_front_distance(self):
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_measure(self.depth, sl.MEASURE.DEPTH)
            depth_numpy = self.depth.get_data()
            depth_numpy = np.where(np.isfinite(depth_numpy), depth_numpy, np.nan)
            height, width = depth_numpy.shape
            crop_left = int(width * 0.35)
            crop_right = int(width * 0.65)
            depth_numpy_far = depth_numpy[:, crop_left:crop_right]
            if np.isnan(depth_numpy).all():
                self.get_logger().warn("深度矩阵中没有有效值")
                return float('inf')
            else:
                self.front_zed_near = np.nanmin(depth_numpy)
                self.front_zed_far = np.nanmin(depth_numpy_far)
                self.left_zed = np.nanmean(depth_numpy[:, :crop_left])
                self.right_zed = np.nanmean(depth_numpy[:, crop_right:])
                min_depth = 0.0
                if self.front_zed_far < 1.0:
                    min_depth = self.front_zed_near
                elif self.front_zed_far >= 1.0 and self.front_zed_near > 0.7:
                    min_depth = self.front_zed_near
                else:
                    min_depth = self.front_zed_near
                return min_depth
        else:
            self.get_logger().warn("无法捕获 ZED 深度数据")
            return float('inf')

    def control_loop(self):
        # PID参数
        linear_kp = 1.8
        linear_ki = 0.0
        linear_kd = 0.5
        angular_kp = 2.0
        angular_ki = 0.0
        angular_kd = 0.3

        prev_linear_error = 0.0
        sum_linear_error = 0.0
        prev_angular_error = 0.0
        sum_angular_error = 0.0

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if not self.has_goal or self.target_point is None:
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                continue
            if self.current_pose is None:
                self.get_logger().warn("当前位姿未更新，等待中...")
                continue

            dx = self.target_point.x - self.current_pose.position.x
            dy = self.target_point.y - self.current_pose.position.y
            distance = math.hypot(dx, dy)
            angle_to_goal = math.atan2(dy, dx)
            q = self.current_pose.orientation
            yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0-2.0*(q.y*q.y + q.z*q.z))
            angle_error = angle_to_goal - yaw
            angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

            front_distance = self.get_front_distance()
            base_safe_distance = 0.85
            max_safe_distance = 2.5
            speed_factor = abs(self.current_speed) / 50.0
            safe_distance = base_safe_distance + speed_factor * (max_safe_distance - base_safe_distance)
            safe_distance = max(0.5, min(safe_distance, max_safe_distance))

            # 避障逻辑
            if self.get_front_distance() < safe_distance:
                left_distance = getattr(self, 'left_distance', 1.0)
                right_distance = getattr(self, 'right_distance', 1.0)
                left_zed = getattr(self, 'left_zed', 1.0)
                right_zed = getattr(self, 'right_zed', 1.0)
                left_score = left_zed * 0.4 + left_distance * 0.6
                right_score = right_zed * 0.4 + right_distance * 0.6
                turn_direction = "left" if (left_score - right_score) > 0 else "right"
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 4.0 if turn_direction == "left" else -4.0
                self.cmd_vel_pub.publish(twist)
                self.get_logger().info(f"避障中，方向: {turn_direction}，前方距离: {front_distance:.2f}，安全距离: {safe_distance:.2f}")
                self.avoid_obstacle = True
                continue

            # 到达目标
            if distance < 0.15:
                twist = Twist()
                self.cmd_vel_pub.publish(twist)
                self.get_logger().info("到达目标点，等待新目标...")
                self.get_logger().info(
                    f"当前位置: ({self.current_pose.position.x:.3f},{self.current_pose.position.y:.3f}), "
                    f"到达目标点，等待新目标..."
                )
                self.has_goal = False
                continue

            # PID控制
            linear_error = distance
            sum_linear_error += linear_error
            d_linear_error = linear_error - prev_linear_error
            linear_speed = linear_kp * linear_error + linear_ki * sum_linear_error + linear_kd * d_linear_error
            prev_linear_error = linear_error

            angular_error = angle_error
            sum_angular_error += angular_error
            d_angular_error = angular_error - prev_angular_error
            angular_speed = angular_kp * angular_error + angular_ki * sum_angular_error + angular_kd * d_angular_error
            prev_angular_error = angular_error

            linear_speed = max(min(linear_speed, 4.0), -4.0)
            self.current_speed = linear_speed
            angular_speed = max(min(angular_speed, 6.0), -6.0)

            twist = Twist()
            if self.avoid_obstacle and self.avoid_obstacle_count <= 20:
                self.avoid_obstacle_count += 1
                twist.linear.x = linear_speed
                twist.angular.z = 0.0
                if self.avoid_obstacle_count == 20:
                    self.avoid_obstacle = False
                    self.avoid_obstacle_count = 0
                    self.get_logger().info("避障结束，恢复正常控制")

            else:
                if abs(angle_error) > 0.3:
                    twist.linear.x = 0.0
                    twist.angular.z = angular_speed
                else:
                    twist.linear.x = linear_speed
                    twist.angular.z = 0.0

            self.cmd_vel_pub.publish(twist)
            self.get_logger().info(
                f"当前位置: ({self.current_pose.position.x:.3f},{self.current_pose.position.y:.3f}), "
                f"目标:({self.target_point.x:.3f},{self.target_point.y:.3f}), "
                f"速度: {twist.linear.x:.2f}, 角速度: {twist.angular.z:.2f}, "
                f"前方距离: {front_distance:.2f}, 安全距离: {safe_distance:.2f}"
            )

def main(args=None):
    rclpy.init(args=args)
    node = GoToPoseTopicNode()
    try:
        node.control_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.zed.close()
        node.get_logger().info("ZED 相机已关闭")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()