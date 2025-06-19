#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String, Bool, Int32
from geometry_msgs.msg import Twist, Point
import math
import time  # 添加时间模块
import pyzed.sl as sl  # 导入 ZED SDK
import numpy as np
from nav_msgs.msg import Odometry

class AutoDriveNode(Node):
    def __init__(self):
        super().__init__('search_fire_node')

        # 发布 /cmd_vel 话题
        self.goal_publisher = self.create_publisher(Point, '/goal_pose', 10)

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
        # self.zed = sl.Camera()
        # self.init_zed_camera()

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

        self.target_x = None  # 目标点
        self.target_y = None  # 目标点
        self.current_position = None  # 当前位姿
        self.current_orientation = None  # 当前姿态
    
    def odom_callback(self, msg):
        """
        处理 /Odometry 消息，保存位置和姿态
        """
        self.current_position = msg.pose.pose.position
        self.current_orientation = msg.pose.pose.orientation
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
            crop_left = int(width * 0.30)  # 左边界，裁剪掉 30%
            crop_right = int(width * 0.70)  # 右边界，裁剪掉 30%

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
                    min_depth = self.front_zed_far
                else:
                    min_depth = self.front_zed_near
                return min_depth
        else:
            self.get_logger().warn("无法捕获 ZED 深度数据")
            return float('inf')  # 如果无法捕获数据，返回无穷大

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
        self.front_avg = min(front_distances)
        self.left_distance = sum(left_distances) / len(left_distances)
        self.right_distance = sum(right_distances) / len(right_distances)
        # self.left_distance = min(left_distances)
        # self.right_distance = min(right_distances)
    
    def control_loop(self):
        # 1. 记录起始点
        if not hasattr(self, 'start_position') and self.current_position is not None:
            self.start_position = (self.current_position.x, self.current_position.y)
            self.get_logger().info(f"记录起始点: {self.start_position}")

        # 2. 已到火源附近，静止
        if self.count_above_1000 > 300 and self.current_position is not None:
            if not hasattr(self, 'fire_position'):
                self.fire_position = (self.current_position.x, self.current_position.y)
                self.get_logger().info(f"到达火源附近，记录火源位置: {self.fire_position}")
            # 停止运动
            self.target_x = None
            self.target_y = None
            self.goal_publisher.publish(Point(x=float('nan'), y=float('nan'), z=0.0))
            return

        # 3. 搜索模式：自主探索
        if self.search_mode:
            now = time.time()
            if self.search_mode_start_time is not None:
                elapsed = now - self.search_mode_start_time
            else:
                elapsed = 0

            # 只有超过5秒才继续探索
            if elapsed < 5:
                self.target_x = None
                self.target_y = None
                self.goal_publisher.publish(Point(x=float('nan'), y=float('nan'), z=0.0))
                self.get_logger().info(f"探索模式已进入 {elapsed:.1f} 秒，未到5秒不探索")
                return

            # latest_distances: 21个方向的距离，取最大值方向
            if self.latest_distances is not None and self.current_position is not None:
                max_idx = int(np.argmax(self.latest_distances))
                angle_step = math.pi / 20  # 180度/20
                yaw = 0.0
                q = self.current_orientation
                if q is not None:
                    yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0-2.0*(q.y*q.y + q.z*q.z))
                target_angle = yaw + (max_idx - 10) * angle_step
                step = 2.0  # 每次前进2米
                goal_x = self.current_position.x + step * math.cos(target_angle)
                goal_y = self.current_position.y + step * math.sin(target_angle)

                # 判断是否需要发布新目标点
                publish_new_goal = False
                # 初始化记录
                if not hasattr(self, 'last_goal'):
                    self.last_goal = (goal_x, goal_y)
                    self.goal_publish_time = now
                    publish_new_goal = True
                else:
                    # 距离上次目标点的距离
                    dist_to_goal = math.hypot(self.current_position.x - self.last_goal[0],
                                            self.current_position.y - self.last_goal[1])
                    # 距离目标点小于0.5米，或超时10秒未到达，则发布新目标点
                    if dist_to_goal < 0.5 or (now - self.goal_publish_time) > 10:
                        publish_new_goal = True

                if publish_new_goal:
                    self.last_goal = (goal_x, goal_y)
                    self.goal_publish_time = now
                    self.goal_publisher.publish(Point(x=goal_x, y=goal_y, z=0.0))
                    self.get_logger().info(f"搜索模式: 选择方向{max_idx}, 发布新目标点({goal_x:.2f}, {goal_y:.2f})")
                else:
                    self.get_logger().info(f"搜索模式: 保持目标点({self.last_goal[0]:.2f}, {self.last_goal[1]:.2f})，距离目标{dist_to_goal:.2f}米")
            return

        # 4. 云台已指向火源方向，沿该方向靠近
        if not self.search_mode and self.current_position is not None:
            # target_horizontal_position: 云台指向的角度（假设为相对机器人正前方的角度，单位弧度）
            # 你需要根据实际协议确认这个角度的定义
            yaw = 0.0
            q = self.current_orientation
            if q is not None:
                yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0-2.0*(q.y*q.y + q.z*q.z))
            # 机器人前进方向 = 当前朝向 + 云台偏转角
            target_angle = yaw + self.target_horizontal_position*math.pi/180.0
            step = 1.0  # 每次靠近0.7米
            self.target_x = self.current_position.x + step * math.cos(target_angle)
            self.target_y = self.current_position.y + step * math.sin(target_angle)
            self.goal_publisher.publish(Point(x=self.target_x, y=self.target_y, z=0.0))
            self.get_logger().info(f"靠近火源: 云台角度{self.target_horizontal_position:.2f}，发布目标点({self.target_x:.2f}, {self.target_y:.2f})")
    
    def stop(self):
        """
        停止
        """
        self.target_x = None
        self.target_y = None
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
        node.get_logger().info("ZED 相机已关闭")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()