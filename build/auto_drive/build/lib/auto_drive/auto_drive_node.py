#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String, Bool, Int32
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import math
import time  # 添加时间模块
import pyzed.sl as sl  # 导入 ZED SDK
import numpy as np
from nav_msgs.msg import Odometry

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

        # 订阅 /count_above_1000 话题
        self.count_above_1000_subscription = self.create_subscription(
            Int32,
            '/count_above_1000',
            self.count_above_1000_callback,
            10
        )

        # 订阅 /Odometry 话题
        self.odom_sub = self.create_subscription(
            Odometry,
            '/Odometry',
            self.odom_callback,
            10
        )

        # 初始化odom的位姿
        self.odom_position = None
        self.odom_orientation = None

        # 初始化 ZED 相机
        self.zed = sl.Camera()
        self.init_zed_camera()

        # 初始化状态
        self.base_safe_distance = 0.85  # 基础安全距离，单位：米
        self.base_speed = 1.5  # 基础速度，单位：米/秒
        self.max_speed = 5.0  # 最大速度
        self.max_safe_distance = 2.5  # 最大安全距离
        self.safe_distance = 0.85  # 安全距离，单位：米
        self.speed = 1.5  # 当前速度
        self.latest_distances = None  # 存储最新的激光雷达数据
        self.front_avg = 0.0  # 前方距离
        self.front_zed = 0.0  # 前方距离
        self.front_zed_near = 0.0  # 前方较近距离
        self.front_zed_far = 0.0  # 前方较远距离
        self.left_zed = 0.0  # 左侧距离
        self.right_zed = 0.0 # 右侧距离
        self.left_distance = 0.0  # 左边距离
        self.right_distance = 0.0  # 右边距离

        # 搜索模式状态
        self.search_mode = True  # 初始状态为寻找模式
        self.search_mode_start_time = None  # 记录进入搜索模式的时间
        self.avoid_obstacle = False  # 避障状态

        # 舵机指向的位置
        self.target_horizontal_position = 0.0
        self.target_vertical_position = 0.0

        self.count_above_1000 = 0
    
    def odom_callback(self, msg):
        """
        处理 /Odometry 消息，保存位置和姿态
        """
        self.odom_position = msg.pose.pose.position
        self.odom_orientation = msg.pose.pose.orientation
        # self.get_logger().info(
        #     f"收到Odom: pos=({self.odom_position.x:.3f}, {self.odom_position.y:.3f}, {self.odom_position.z:.3f}), "
        #     f"ori=({self.odom_orientation.x:.3f}, {self.odom_orientation.y:.3f}, {self.odom_orientation.z:.3f}, {self.odom_orientation.w:.3f})"
        # )

    def count_above_1000_callback(self, msg):
        """
        处理 /count_above_1000 话题的回调函数
        """
        self.count_above_1000 = msg.data
        self.get_logger().info(f"接收到 /count_above_1000 数据: {self.count_above_1000}")

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
                    if self.search_mode == False:
                        self.target_horizontal_position = position
                    else:
                        self.target_horizontal_position = 0.0
                elif servo_id == 1:  # 垂直舵机
                    if self.search_mode == False:
                        self.target_vertical_position = position
                    else:
                        self.target_vertical_position = 0.0

                # self.get_logger().info(f"舵机指向更新: 水平角度={self.target_horizontal_position}, 垂直角度={self.target_vertical_position}")
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
            if self.search_mode_start_time is None:
                self.search_mode_start_time = time.time()  # 记录进入搜索模式的时间
        elif self.search_mode is False:
            self.search_mode_start_time = time.time()

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
                    self.get_logger().warn("前方距离过近，使用较近的深度值")
                    min_depth = self.front_zed_near
                elif self.front_zed_far >= 1.0 and self.front_zed_near > 0.7:
                    self.get_logger().info("前方距离正常，使用较远的深度值")
                    min_depth = self.front_zed_near
                else:
                    min_depth = self.front_zed_near
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
        # 如果处于搜索模式，检查持续时间
        if self.search_mode:
            self.get_logger().info(f"搜索时间: {self.search_mode_start_time}")
            if self.search_mode_start_time is not None:
                elapsed_time = time.time() - self.search_mode_start_time
                self.get_logger().info(f"搜索模式持续时间: {elapsed_time:.2f} 秒")
                if elapsed_time > 15.0:  # 搜索模式超过 15 秒
                    self.get_logger().info("搜索模式超过 15 秒，触发掉头")
                    self.turn_around()
                    self.search_mode_start_time = time.time()  # 重置时间
            return

        # 如果检测到起火点，立即停止运动
        if self.count_above_1000 >= 5000:
            self.stop()
            self.get_logger().info("找到起火点，停止运动")
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

        # 检查前方是否安全
        if self.front_zed > self.safe_distance and self.count_above_1000 < 4000:
            # 前方安全，继续前进
            t1 = time.time()
            while self.front_zed > self.safe_distance and self.avoid_obstacle is True and time.time() - t1 < 2.0:
                self.drive_forward(self.speed)
                self.get_logger().info(f"避障之后前进一小段")
                rclpy.spin_once(self, timeout_sec=0.1)

                # 在循环中频繁检查搜索模式
                if self.search_mode:
                    self.stop()
                    return
                if self.count_above_1000 >= 4000:
                    self.stop()
                    self.get_logger().info("找到起火点，停止运动")
                    return

                self.front_zed = self.get_front_distance()

            self.avoid_obstacle = False  # 重置避障状态

            if self.search_mode:
                self.stop()
                self.get_logger().info("搜索模式中，停止运动")
                return
            if self.count_above_1000 >= 4000:
                    self.stop()
                    self.get_logger().info("找到起火点，停止运动")
                    return

            self.move_to_target()
        elif self.front_zed <= self.safe_distance and self.count_above_1000 < 5000:
            # 前方不安全，停止并转向
            self.stop()
            time.sleep(0.2)
            self.avoid_obstacle = True

            turn_direction = "left" if ((self.left_zed * 0.4 + self.left_distance * 0.6) - (self.right_zed * 0.4 + self.right_distance * 0.6)) > 0 else "right"

            # 持续转向，直到前方距离大于安全距离
            while self.front_zed <= self.safe_distance and self.count_above_1000 < 5000:
                if turn_direction == "left":
                    self.turn_left()
                else:
                    self.turn_right()
                
                # 如果检测到起火点，立即停止运动
                if self.count_above_1000 >= 4000:
                    self.stop()
                    self.get_logger().info("找到起火点，停止运动")
                    return
                # 在循环中频繁检查搜索模式
                if self.search_mode:
                    self.stop()
                    return

                # 等待新的激光雷达数据
                rclpy.spin_once(self, timeout_sec=0.1)

                # 在循环中频繁检查搜索模式
                if self.search_mode:
                    self.stop()
                    return
                if self.count_above_1000 >= 4000:
                    self.stop()
                    self.get_logger().info("找到起火点，停止运动")
                    return

                # 打印调整过程中的前方距离
                self.get_logger().info(f"避障调整中，前方距离: {self.front_zed:.2f} 米，安全距离: {self.safe_distance:.2f} 米， 左侧距离: {self.left_zed * 0.4 + self.left_distance * 0.6:.2f} 米，右侧距离: {self.right_zed * 0.4 + self.right_distance * 0.6:.2f} 米")
                self.front_zed = self.get_front_distance()

            # 停止转向，停顿0.2秒
            self.stop()
            time.sleep(0.2)
    
    def move_to_target(self):
        """
        根据舵机指向的位置移动机器人
        """
        # 简单示例：根据水平角度调整机器人方向
        if self.target_horizontal_position > 30.0:  # 偏左
            t1 = time.time()
            t2 = abs(self.target_horizontal_position)*0.01
            while time.time() - t1 < t2:
                self.get_logger().info(f"左转时间: {t2:.2f} 秒") 
                self.turn_left()
                rclpy.spin_once(self, timeout_sec=0.1)
                if self.search_mode:
                    self.stop()
                    self.get_logger().info("搜索模式中，停止运动")
                    return
                if self.count_above_1000 >= 4000:
                    self.stop()
                    self.get_logger().info("找到起火点，停止运动")
                    return
        elif self.target_horizontal_position < -30.0:  # 偏右
            t1 = time.time()
            t2 = abs(self.target_horizontal_position)*0.01
            while time.time() - t1 < t2:
                self.get_logger().info(f"右转时间: {t2:.2f} 秒") 
                self.turn_right()
                rclpy.spin_once(self, timeout_sec=0.1)
                if self.search_mode:
                    self.stop()
                    self.get_logger().info("搜索模式中，停止运动")
                    return
                if self.count_above_1000 >= 4000:
                    self.stop()
                    self.get_logger().info("找到起火点，停止运动")
                    return
        else:
            self.drive_forward(self.speed)
            if self.count_above_1000 >= 4000:
                self.stop()
                self.get_logger().info("找到起火点，停止运动")
                return
    
    def calculate_dynamic_speed_and_distance(self, front_distance):
        """
        根据前方距离动态计算速度和安全距离
        :param front_distance: 前方距离
        :return: 动态速度和安全距离
        """
        self.get_logger().info(
            f"前方最小距离: {self.front_zed:.2f} 米, 左侧距离: {self.left_zed*0.4 + self.left_distance*0.6:.2f} 米, 右侧距离: {self.right_zed*0.4+self.right_distance*0.6:.2f} 米"
        )

        # 计算动态速度和安全距离
        speed = self.base_speed + (front_distance - self.safe_distance)* 1.0 * (self.max_speed - self.base_speed)
        safe_distance = self.base_safe_distance + abs(front_distance - self.safe_distance)* 0.2 * (self.max_safe_distance - self.base_safe_distance)
        self.speed = max(1.0, min(speed, self.max_speed))  # 限制速度在 0 到 max_speed 之间
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
        # self.get_logger().info("左转中...")

    def turn_right(self):
        """
        右转
        """
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = -4.0  # 设置右转角速度
        self.cmd_vel_publisher.publish(twist)
        # self.get_logger().info("右转中...")

    def turn_around(self):
        """
        掉头
        """
        self.get_logger().info("触发掉头操作...")
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 6.0  # 设置较大的角速度进行掉头
        turn_duration = 4.0  # 掉头持续时间（秒）
        start_time = time.time()
        while time.time() - start_time < turn_duration:
            self.cmd_vel_publisher.publish(twist)
            time.sleep(0.1)  # 控制发布频率

        # 停止机器人
        self.stop()
        self.get_logger().info("掉头完成")

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