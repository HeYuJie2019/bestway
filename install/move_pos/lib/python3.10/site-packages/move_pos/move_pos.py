import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from move_pos_interfaces.action import GoToPose
from rclpy.action import ActionServer
from std_msgs.msg import Float32MultiArray
import math
import time
import pyzed.sl as sl  # 导入 ZED SDK
import numpy as np
import asyncio
from rclpy.executors import MultiThreadedExecutor

class GoToPoseActionServer(Node):
    def __init__(self):
        super().__init__('move_pos_action_server')
        self._action_server = ActionServer(
            self,
            GoToPose,
            'go_to_pose',
            self.execute_callback)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/Odometry', self.odom_callback, 10)
        self.current_pose = None
        # 订阅激光雷达距离数据
        self.lidar_subscription = self.create_subscription(
            Float32MultiArray,
            'horizontal_distances',
            self.lidar_callback,
            10
        )
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
    
    def init_zed_camera(self):
        """
        初始化 ZED 深度相机
        """
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720  # 设置分辨率
        init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # 设置深度模式
        init_params.coordinate_units = sl.UNIT.METER  # 深度单位为米

        status = self.zed.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f"无法打开 ZED 相机: {status}")
            exit(1)

        # 创建深度图像对象
        self.depth = sl.Mat()

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.get_logger().info(f"收到里程计: x={self.current_pose.position.x:.3f}, y={self.current_pose.position.y:.3f}")
    
    def lidar_callback(self, msg):
        """
        激光雷达数据回调函数
        """
        self.latest_distances = msg.data  # 实时更新激光雷达数据
        num_points = len(self.latest_distances)

        if num_points != 21:
            self.get_logger().warn("接收到的水平距离数据数量与预期不符！")
            return

        # 获取前方、左侧和右侧的平均距离
        front_distances = self.latest_distances[8:13]  # 正前方范围
        left_distances = self.latest_distances[13:]  # 左侧范围
        right_distances = self.latest_distances[:8]  # 右侧范围

        # 更新前方距离为前方区间中的最小值
        self.front_distance = min(front_distances)
        self.left_distance = sum(left_distances) / len(left_distances)
        self.right_distance = sum(right_distances) / len(right_distances)
        # self.left_distance = min(left_distances)
        # self.right_distance = min(right_distances)

    def get_front_distance(self):
        """
        获取前方距离（使用 ZED 深度相机）
        """
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            # 获取深度图
            self.zed.retrieve_measure(self.depth, sl.MEASURE.DEPTH)

            # 将深度数据转换为 numpy 数组
            depth_numpy = self.depth.get_data()

            # 过滤无效值（例如 -inf, inf）
            depth_numpy = np.where(np.isfinite(depth_numpy), depth_numpy, np.nan)

            # 裁剪矩阵，只保留中间部分（左右裁剪）
            height, width = depth_numpy.shape
            crop_left = int(width * 0.35)  # 左边界，裁剪掉 30%
            crop_right = int(width * 0.65)  # 右边界，裁剪掉 30%

            depth_numpy_far = depth_numpy[:, crop_left:crop_right]  # 仅裁剪列，保留所有行

            # 获取深度矩阵的最小值（忽略 NaN）
            if np.isnan(depth_numpy).all():
                self.get_logger().warn("深度矩阵中没有有效值")
                return float('inf')  # 如果没有有效值，返回无穷大
            else:
                self.front_zed_near = np.nanmin(depth_numpy)
                self.front_zed_far = np.nanmin(depth_numpy_far)
                self.left_zed = np.nanmean(depth_numpy[:, :crop_left])
                self.right_zed = np.nanmean(depth_numpy[:, crop_right:])
                min_depth = 0.0
                if self.front_zed_far < 1.0:
                    # self.get_logger().warn("前方距离过近，使用较近的深度值")
                    min_depth = self.front_zed_near
                elif self.front_zed_far >= 1.0 and self.front_zed_near > 0.7:
                    # self.get_logger().info("前方距离正常，使用较远的深度值")
                    min_depth = self.front_zed_near
                else:
                    min_depth = self.front_zed_near
                return min_depth
        else:
            self.get_logger().warn("无法捕获 ZED 深度数据")
            return float('inf')  # 如果无法捕获数据，返回无穷大

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'接收到目标: ({goal_handle.request.target_x}, {goal_handle.request.target_y})')
        success = False
        message = ""

        # PID参数
        linear_kp = 0.8
        linear_ki = 0.0
        linear_kd = 0.2
        angular_kp = 1.2
        angular_ki = 0.0
        angular_kd = 0.3

        prev_linear_error = 0.0
        sum_linear_error = 0.0
        prev_angular_error = 0.0
        sum_angular_error = 0.0

        while rclpy.ok():
            if self.current_pose is None:
                self.get_logger().warn("当前位姿未更新，等待中...")
                rclpy.spin_once(self, timeout_sec=0.1)
                continue

            # 计算当前位置与目标点的距离和角度
            dx = goal_handle.request.target_x - self.current_pose.position.x
            dy = goal_handle.request.target_y - self.current_pose.position.y
            distance = math.hypot(dx, dy)
            angle_to_goal = math.atan2(dy, dx)
            # 获取当前朝向
            q = self.current_pose.orientation
            yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0-2.0*(q.y*q.y + q.z*q.z))

            # 角度误差归一化到[-pi, pi]
            angle_error = angle_to_goal - yaw
            angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

            # 动态调整安全距离：结合前方距离和当前速度
            front_distance = self.get_front_distance()
            base_safe_distance = 0.85
            max_safe_distance = 2.5
            # 速度因子，速度越快安全距离越大
            speed_factor = abs(self.current_speed) / 50.0  # 4.0为最大线速度
            safe_distance = base_safe_distance + speed_factor * (max_safe_distance - base_safe_distance)
            safe_distance = max(0.5, min(safe_distance, max_safe_distance))

            # 避障逻辑
            if self.get_front_distance() < safe_distance:
                # 判断左右距离，选择更优方向避障
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
                rclpy.spin_once(self, timeout_sec=0.1)
                continue

            # 到达目标
            if distance < 0.15:
                twist = Twist()
                self.cmd_vel_pub.publish(twist)  # 停止
                success = True
                message = "到达目标点"
                break

            # PID控制
            # 线速度PID
            linear_error = distance
            sum_linear_error += linear_error
            d_linear_error = linear_error - prev_linear_error
            linear_speed = linear_kp * linear_error + linear_ki * sum_linear_error + linear_kd * d_linear_error
            prev_linear_error = linear_error

            # 角速度PID
            angular_error = angle_error
            sum_angular_error += angular_error
            d_angular_error = angular_error - prev_angular_error
            angular_speed = angular_kp * angular_error + angular_ki * sum_angular_error + angular_kd * d_angular_error
            prev_angular_error = angular_error

            # 限制最大速度
            linear_speed = max(min(linear_speed, 4.0), -4.0)
            self.current_speed = linear_speed
            angular_speed = max(min(angular_speed, 6.0), -6.0)

            twist = Twist()
            # 如果角度误差较大，优先原地转向
            if abs(angle_error) > 0.3:
                twist.linear.x = 0.0
                twist.angular.z = angular_speed
            else:
                twist.linear.x = linear_speed
                twist.angular.z = angular_speed

            self.cmd_vel_pub.publish(twist)
            self.get_logger().info(f"当前x位置: {self.current_pose.position.x}, 当前y位置: {self.current_pose.position.y}, 当前速度: {linear_speed:.2f}, 角速度: {angular_speed:.2f}, 前方距离: {front_distance:.2f}, 安全距离: {safe_distance:.2f}")
            rclpy.spin_once(self, timeout_sec=0.1)

        goal_handle.succeed()
        result = GoToPose.Result()
        result.success = success
        result.message = message
        return result

def main(args=None):
    rclpy.init(args=args)
    node = GoToPoseActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 确保在退出时关闭 ZED 相机
        node.zed.close()
        node.get_logger().info("ZED 相机已关闭")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()