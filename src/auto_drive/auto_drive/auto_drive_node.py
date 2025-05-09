#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String, Bool
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import math
import time  # 添加时间模块
import pyzed.sl as sl  # 导入 ZED SDK
import numpy as np

class AutoDriveNode(Node):
    def __init__(self):
        super().__init__('auto_drive_node')

        # 发布 /cmd_vel 话题
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # 订阅激光雷达距离数据
        self.lidar_subscription = self.create_subscription(
            Float32MultiArray,
            'horizontal_distances',
            self.lidar_callback,
            10
        )

        # 订阅 /servo_position 话题
        self.servo_position_subscription = self.create_subscription(
            String,
            '/servo_position',
            self.servo_position_callback,
            100
        )

        # 订阅 /search_mode_status 话题
        self.search_mode_subscription = self.create_subscription(
            Bool,
            '/search_mode_status',
            self.search_mode_callback,
            100
        )

        # 初始化 ZED 相机
        self.zed = sl.Camera()
        self.init_zed_camera()

        # 初始化状态
        self.base_safe_distance = 0.5  # 基础安全距离，单位：米
        self.base_speed = 1.5  # 基础速度，单位：米/秒
        self.max_speed = 5.0  # 最大速度
        self.max_safe_distance = 2.5  # 最大安全距离
        self.safe_distance = 0.5  # 安全距离，单位：米
        self.speed = 1.5  # 当前速度
        self.latest_distances = None  # 存储最新的激光雷达数据
        self.front_avg = 0.0  # 前方距离
        self.front_zed = 0.0  # 前方距离
        self.left_distance = 0.0  # 左边距离
        self.right_distance = 0.0  # 右边距离

        # 搜索模式状态
        self.search_mode = False

        # 舵机指向的位置
        self.target_horizontal_position = 0.0
        self.target_vertical_position = 0.0

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
    
    def servo_position_callback(self, msg):
        """
        处理 /servo_position 话题的回调函数
        """
        try:
            # 解析舵机指向的位置
            servo_data = msg.data.split(':')
            if len(servo_data) == 2:
                servo_id = int(servo_data[0])
                position = float(servo_data[1])

                if servo_id == 0:  # 水平舵机
                    self.target_horizontal_position = position
                elif servo_id == 1:  # 垂直舵机
                    self.target_vertical_position = position

                self.get_logger().info(f"舵机指向更新: 水平角度={self.target_horizontal_position}, 垂直角度={self.target_vertical_position}")
        except Exception as e:
            self.get_logger().error(f"解析舵机指向失败: {e}")

    def search_mode_callback(self, msg):
        """
        处理 /search_mode_status 话题的回调函数
        """
        self.search_mode = msg.data
        if self.search_mode:
            self.get_logger().info("进入搜索模式，机器人停止运动")
            self.stop()

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

            # 获取深度矩阵的最小值（忽略 NaN）
            if np.isnan(depth_numpy).all():
                self.get_logger().warn("深度矩阵中没有有效值")
                return float('inf')  # 如果没有有效值，返回无穷大
            else:
                min_depth = np.nanmin(depth_numpy)
                return min_depth
        else:
            self.get_logger().warn("无法捕获 ZED 深度数据")
            return float('inf')  # 如果无法捕获数据，返回无穷大

    def imu_callback(self, msg):
        """
        IMU 数据回调函数，用于计算累计旋转角度
        """
        # 获取当前时间戳
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.last_time is None:
            # 如果是第一次接收 IMU 数据，初始化时间戳
            self.last_time = current_time
            return

        # 计算时间间隔
        delta_time = current_time - self.last_time
        self.last_time = current_time

        # 获取绕 Z 轴的角速度（单位：弧度/秒）
        angular_velocity_z = msg.angular_velocity.z

        # 计算增量角度（单位：度）
        delta_angle = math.degrees(angular_velocity_z * delta_time)

        # 累加旋转角度
        self.total_turn_angle += delta_angle

        self.get_logger().info(f"累计旋转角度: {self.total_turn_angle:.2f}°")

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

        # self.front_avg = sum(front_distances) / len(front_distances)
        # 更新前方距离为前方区间中的最小值
        self.front_avg = min(front_distances)
        # left_avg = sum(left_distances) / len(left_distances)
        # right_avg = sum(right_distances) / len(right_distances)
        self.left_distance = min(left_distances)
        self.right_distance = min(right_distances)
    
    def control_loop(self):
        """
        主控制循环
        """
        # 如果处于搜索模式，停止运动
        if self.search_mode:
            self.stop()
            return

        # 获取前方距离（使用 ZED 深度相机）
        self.front_zed = self.get_front_distance()

        if self.latest_distances is None:
            return  # 如果没有激光雷达数据，什么都不做
        if self.front_zed is None:
            return  # 如果没有深度相机数据，什么都不做
        
        # 动态调整速度和安全距离
        self.calculate_dynamic_speed_and_distance(self.front_zed)
        self.get_logger().info(f"动态调整速度: {self.speed:.2f} m/s, 安全距离: {self.safe_distance:.2f} 米")

        # if self.front_avg > self.safe_distance:
        if self.front_zed > self.safe_distance:
            # 前方安全，继续前进
            self.drive_forward(self.speed)
        else:
            # 前方不安全，停止并转向
            self.stop()
            time.sleep(0.5)

            # 持续转向，直到前方距离大于安全距离
            # while self.front_avg <= self.safe_distance:
            while self.front_zed <= self.safe_distance:
                if self.left_distance > self.right_distance:
                    # 如果左侧距离大于右侧距离，向右转
                    self.turn_left()
                else:
                    self.turn_right()

                # 等待新的激光雷达数据
                rclpy.spin_once(self, timeout_sec=0.1)

                # 打印调整过程中的前方距离
                # self.get_logger().info(f"调整中，前方距离: {self.front_avg:.2f} 米")
                self.get_logger().info(f"调整中，前方距离: {self.front_zed:.2f} 米")
                self.front_zed = self.get_front_distance()
            # 停止转向，停顿0.5秒
            self.stop()
            time.sleep(0.5)
    
    def move_to_target(self):
        """
        根据舵机指向的位置移动机器人
        """
        # 简单示例：根据水平角度调整机器人方向
        if self.target_horizontal_position > 5.0:  # 偏右
            self.turn_right()
        elif self.target_horizontal_position < -5.0:  # 偏左
            self.turn_left()
        else:
            self.drive_forward(self.speed)
    
    def calculate_dynamic_speed_and_distance(self, front_distance):
        """
        根据前方距离动态计算速度和安全距离
        :param front_distance: 前方距离
        :return: 动态速度和安全距离
        """
        self.get_logger().info(
            # f"前方平均距离: {self.front_avg:.2f} 米, 左侧平均距离: {left_avg:.2f} 米, 右侧平均距离: {right_avg:.2f} 米"
            f"前方最小距离: {self.front_zed:.2f} 米, 左侧最小距离: {self.left_distance:.2f} 米, 右侧最小距离: {self.right_distance:.2f} 米"
        )

        # 计算动态速度和安全距离
        speed = self.base_speed + (front_distance - self.safe_distance)* 0.1 * (self.max_speed - self.base_speed)
        safe_distance = self.base_safe_distance + (front_distance - self.safe_distance)* 0.1 * (self.max_safe_distance - self.base_safe_distance)
        self.speed = max(0.0, min(speed, self.max_speed))  # 限制速度在 0 到 max_speed 之间
        self.safe_distance = max(0.0, min(safe_distance, self.max_safe_distance))  # 限制安全距离在 0 到 max_safe_distance 之间

    def drive_forward(self, speed):
        """
        前进
        """
        twist = Twist()
        twist.linear.x = float(speed)
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info(f"前进中，速度: {speed:.2f} m/s")

    def turn_left(self):
        """
        左转
        """
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 4.0  # 设置左转角速度
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info("左转中...")

    def turn_right(self):
        """
        右转
        """
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = -4.0  # 设置右转角速度
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info("右转中...")

    def stop(self):
        """
        停止
        """
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info("停止中...")

def main(args=None):
    rclpy.init(args=args)
    node = AutoDriveNode()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.control_loop()
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