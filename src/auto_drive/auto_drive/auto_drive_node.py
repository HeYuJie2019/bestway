#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
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

        # 初始化 ZED 相机
        self.zed = sl.Camera()
        self.init_zed_camera()

        # 初始化状态
        self.safe_distance = 0.5  # 安全距离，单位：米
        self.latest_distances = None  # 存储最新的激光雷达数据
        self.turning_left = None  # 当前转向方向
        self.front_avg = 0.0  # 前方距离
        self.front_zed = 0.0  # 前方距离

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
        left_avg = min(left_distances)
        right_avg = min(right_distances)

        self.get_logger().info(
            f"前方平均距离: {self.front_avg:.2f} 米, 左侧平均距离: {left_avg:.2f} 米, 右侧平均距离: {right_avg:.2f} 米"
        )

        # 判断转向方向
        if self.front_avg <= self.safe_distance:
            self.turning_left = left_avg > right_avg
    
    def control_loop(self):
        """
        主控制循环
        """
        # 获取前方距离（使用 ZED 深度相机）
        self.front_zed = self.get_front_distance()

        if self.latest_distances is None:
            return  # 如果没有激光雷达数据，什么都不做

        # if self.front_avg > self.safe_distance:
        if self.front_zed > self.safe_distance:
            # 前方安全，继续前进
            self.drive_forward()
        else:
            # 前方不安全，停止并转向
            self.stop()
            time.sleep(2)

            # 持续转向，直到前方距离大于安全距离
            # while self.front_avg <= self.safe_distance:
            while self.front_zed <= self.safe_distance:
                if self.turning_left:
                    self.turn_left()
                else:
                    self.turn_right()

                # 等待新的激光雷达数据
                rclpy.spin_once(self, timeout_sec=0.1)

                # 打印调整过程中的前方距离
                # self.get_logger().info(f"调整中，前方距离: {self.front_avg:.2f} 米")
                self.get_logger().info(f"调整中，前方距离: {self.front_zed:.2f} 米")

            # 停止转向，停顿1秒
            self.stop()
            time.sleep(1)

    def drive_forward(self):
        """
        前进
        """
        twist = Twist()
        twist.linear.x = 2.0  # 设置前进速度
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info("前进中...")

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