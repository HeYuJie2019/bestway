import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from livox_ros_driver2.msg import CustomMsg
import math
import time
import numpy as np

class GoToPoseTopicNode(Node):
    def __init__(self):
        super().__init__('move_pos_topic_node')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/Odometry', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(Point, '/goal_pose', self.goal_callback, 10)
        self.lidar_subscription = self.create_subscription(
            CustomMsg,
            '/livox/lidar',
            self.lidar_callback,
            10
        )
        self.current_pose = None
        self.target_point = None
        self.has_goal = False
        self.left_distance = 0.0
        self.right_distance = 0.0
        self.front_distance = 0.0
        self.current_speed = 0.0
        self.avoid_obstacle = False
        self.avoid_obstacle_count = 0
        

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        # self.get_logger().info(f"收到里程计: x={self.current_pose.position.x:.3f}, y={self.current_pose.position.y:.3f}")

    def goal_callback(self, msg):
        self.target_point = msg
        self.has_goal = True
        # self.get_logger().info(f"收到新目标点: x={msg.x:.3f}, y={msg.y:.3f}")

    def lidar_callback(self, msg):
        # 旋转角度（前倾45°，绕y轴-45°）
        theta = math.radians(-45)
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)
        front_min = float('inf')
        left_min = float('inf')
        right_min = float('inf')
        front_count = left_count = right_count = 0
        total_points = len(msg.points)
        filtered_points = 0
        for idx, point in enumerate(msg.points):
            x, y, z = point.x, point.y, point.z
            # 绕y轴-45°旋转，适配前倾
            x_p = x * cos_theta + z * sin_theta
            y_p = y
            z_p = -x * sin_theta + z * cos_theta
            # 放宽z_p阈值，临时注释x_p<=0
            if abs(z_p) > 0.25:
                continue
            filtered_points += 1
            distance = math.hypot(x_p, y_p)
            # 打印前10个点的旋转前后坐标
            if idx < 10:
                self.get_logger().info(f"原始:({x:.2f},{y:.2f},{z:.2f}) 旋转后:({x_p:.2f},{y_p:.2f},{z_p:.2f})")
            # 前方：x'较大且|y'|较小
            if x_p > 0.2 and abs(y_p) < 0.3:
                if distance < front_min:
                    front_min = distance
                front_count += 1
            # 左侧：y'>0.2且|x'|较小
            if y_p > 0.2 and abs(x_p) < 0.5:
                if distance < left_min:
                    left_min = distance
                left_count += 1
            # 右侧：y'<-0.2且|x'|较小
            if y_p < -0.2 and abs(x_p) < 0.5:
                if distance < right_min:
                    right_min = distance
                right_count += 1
        if front_count == 0:
            front_min = float('inf')
        if left_count == 0:
            left_min = float('inf')
        if right_count == 0:
            right_min = float('inf')
        self.front_distance = front_min
        self.left_distance = left_min
        self.right_distance = right_min
        self.get_logger().info(f"点云总数: {total_points}, 过滤后: {filtered_points}, 前方点: {front_count}, 左: {left_count}, 右: {right_count}")
        self.get_logger().info(f"前方距离: {front_min:.2f}, 左侧距离: {left_min:.2f}, 右侧距离: {right_min:.2f}")

    def control_loop(self):
        # PID参数
        linear_kp = 1.8
        linear_ki = 0.0
        linear_kd = 0.5
        angular_kp = 4.0
        angular_ki = 0.0
        angular_kd = 0.6

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
            # 检查目标点有效性
            if math.isnan(self.target_point.x) or math.isnan(self.target_point.y):
                self.has_goal = False
                self.target_point = None
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

            # 仅用雷达前方距离
            front_distance = self.front_distance
            base_safe_distance = 1.2
            max_safe_distance = 2.5
            speed_factor = abs(self.current_speed) / 50.0
            safe_distance = base_safe_distance + speed_factor * (max_safe_distance - base_safe_distance)
            safe_distance = max(0.5, min(safe_distance, max_safe_distance))

            # 避障逻辑
            if front_distance < safe_distance:
                # 只用雷达信息避障
                left_distance = getattr(self, 'left_distance', 1.0)
                right_distance = getattr(self, 'right_distance', 1.0)
                self.get_logger().info(
                    f"左侧距离: {left_distance:.2f}, 右侧距离: {right_distance:.2f}")
                prefer_direction = "left" if angle_error > 0 else "right"
                if left_distance < 0.8 or right_distance < 0.8:
                    turn_direction = "left" if left_distance > right_distance else "right"
                else:
                    turn_direction = prefer_direction
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 4.0 if turn_direction == "left" else -4.0
                self.cmd_vel_pub.publish(twist)
                self.get_logger().info(
                    f"避障中，方向: {turn_direction}，目标在: {prefer_direction}，"
                    f"前方距离: {front_distance:.2f}，安全距离: {safe_distance:.2f}"
                )
                self.avoid_obstacle = True
                continue

            # 到达目标
            if distance < 0.5:
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

            linear_speed = max(min(linear_speed, 3.0), -3.0)
            self.current_speed = linear_speed
            angular_speed = max(min(angular_speed, 6.0), -6.0)

            twist = Twist()
            if abs(angle_error) > 1.5:
                twist.linear.x = 0.0
                twist.angular.z = angular_speed
            else:
                twist.linear.x = linear_speed
                twist.angular.z = 0.0

            self.cmd_vel_pub.publish(twist)
            self.get_logger().info(
                f"角度差: {abs(angle_error):.2f}, "
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
        node.get_logger().info("节点已关闭")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()