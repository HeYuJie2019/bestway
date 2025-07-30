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
import csv
from std_msgs.msg import Float32

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

        # 订阅 /average_temperature 话题
        self.temperature_subscription = self.create_subscription(
            Float32,
            '/average_temperature',
            self.temperature_callback,
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
        self.returning = False  # 是否返回状态

        # 搜索模式状态
        self.search_mode = True  # 初始状态为寻找模式
        self.search_mode_start_time = None  # 记录进入搜索模式的时间
        self.avoid_obstacle = False  # 避障状态

        # 舵机指向的位置
        self.target_horizontal_position = 0.0
        self.target_vertical_position = 0.0
        
        # 云台角度可靠性检测
        self.servo_angle_history = []  # 存储最近的云台角度
        self.max_history_size = 5      # 历史记录最大长度
        self.last_valid_servo_angle = None  # 最后一个有效的云台角度
        self.servo_angle_stable_count = 0   # 云台角度稳定计数

        self.count_above_1000 = 0

        self.target_x = None  # 目标点
        self.target_y = None  # 目标点
        self.current_position = None  # 当前位姿
        self.current_orientation = None  # 当前姿态

        # 轨迹记录
        self.temperature_subscription = None
        self.current_temperature = None
        self.trajectory = []  # 记录轨迹点 (x, y, temp)
        self.trajectory_file = 'trajectory.csv'
        self.last_recorded_point = None
        self.recording = False
        self.returning_path = []

        # 新增：火源丢失等待期相关变量
        self.pending_fire_lost = False  # 是否处于火源丢失等待期
        self.fire_lost_time = None      # 火源丢失的时间戳
        self.fire_lost_temp_goal = None # 火源丢失时的临时目标点
        
        # 新增：记录是否曾经进入过靠近火源模式
        self.has_entered_approach_mode = False  # 是否曾经进入过靠近火源模式
        self.last_approach_direction = None     # 最后一次靠近火源时的大致方向
        self.need_smart_first_goal = False      # 是否需要智能生成第一个探索目标点
        
        # 新增：记录是否曾经出现过count_above_1000大于10的情况
        self.has_count_above_10 = False         # 是否曾经出现过count_above_1000 > 10
        
        # 程序计时功能
        self.start_time = time.time()           # 记录程序开始时间
        self.get_logger().info(f"程序开始运行，记录开始时间: {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(self.start_time))}")

    def calculate_and_log_total_time(self):
        """
        计算并输出程序总耗时
        """
        if hasattr(self, 'start_time'):
            end_time = time.time()
            total_time = end_time - self.start_time
            hours = int(total_time // 3600)
            minutes = int((total_time % 3600) // 60)
            seconds = total_time % 60
            
            time_str = ""
            if hours > 0:
                time_str += f"{hours}小时"
            if minutes > 0:
                time_str += f"{minutes}分钟"
            time_str += f"{seconds:.2f}秒"
            
            self.get_logger().info(f"程序结束，总耗时: {time_str}")
            print(f"程序总耗时: {time_str}")
        else:
            self.get_logger().warn("未找到开始时间，无法计算总耗时")

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
        # 记录是否曾经出现过count_above_1000 > 10的情况
        if self.count_above_1000 > 10:
            self.has_count_above_10 = True
        # self.get_logger().info(f"接收到 /count_above_1000 数据: {self.count_above_1000}")

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
                        # 云台角度可靠性检测
                        self.servo_angle_history.append(position)
                        if len(self.servo_angle_history) > self.max_history_size:
                            self.servo_angle_history.pop(0)
                        
                        # 检测云台角度是否可靠
                        if self.is_servo_angle_reliable(position):
                            self.target_horizontal_position = position
                            self.last_valid_servo_angle = position
                            self.servo_angle_stable_count += 1
                            # self.get_logger().info(f"云台角度有效: {position:.2f}度，稳定次数: {self.servo_angle_stable_count}")
                        else:
                            # 使用最后一个有效角度或继续使用当前角度
                            if self.last_valid_servo_angle is not None and abs(position) < 5.0:
                                # 如果新角度接近0且有历史有效角度，使用历史角度
                                self.target_horizontal_position = self.last_valid_servo_angle
                                self.get_logger().warn(f"云台角度可能不可靠({position:.2f}度)，使用上次有效角度: {self.last_valid_servo_angle:.2f}度")
                            else:
                                self.target_horizontal_position = position
                                self.get_logger().warn(f"云台角度可能不可靠: {position:.2f}度，但无有效历史数据，继续使用")
                            self.servo_angle_stable_count = 0
                    else:
                        self.target_horizontal_position = 0.0
                        self.servo_angle_stable_count = 0
                elif servo_id == 1:  # 垂直舵机
                    if self.search_mode == False:
                        self.target_vertical_position = position
                    else:
                        self.target_vertical_position = 0.0

                # self.get_logger().info(f"舵机指向更新: 水平角度={self.target_horizontal_position}, 垂直角度={self.target_vertical_position}")
        except Exception as e:
            self.get_logger().error(f"解析舵机指向失败: {e}")
    
    def is_servo_angle_reliable(self, current_angle):
        """
        检测云台角度是否可靠
        """
        # 只要不是0度就认为可靠
        if abs(current_angle) > 1e-3:
            return True
        # 如果是0度，则连续多次为0才认为可靠
        zero_count = sum(abs(angle) < 1e-3 for angle in self.servo_angle_history[-5:])
        if zero_count >= 5:
            return True
        return False

    def search_mode_callback(self, msg):
        """
        处理 /search_mode_status 话题的回调函数
        """
        prev_search_mode = self.search_mode
        self.search_mode = msg.data
        if self.search_mode:
            # 检查是否是从靠近火源突然退出，且不在返回模式中
            if prev_search_mode is False and not (hasattr(self, 'returning') and self.returning):
                # 进入丢失火源等待期，根据当前位置和激光数据智能选择目标点
                self.lost_fire_waiting = True
                self.lost_fire_time = time.time()
                # 标记需要智能生成第一个探索目标点
                self.need_smart_first_goal = True
                # 智能选择丢失火源后的目标点
                if hasattr(self, 'approach_goal') and self.approach_goal is not None and self.current_position is not None:
                    goal_x, goal_y = self.approach_goal
                    current_x, current_y = self.current_position.x, self.current_position.y
                    dist_to_approach_goal = math.hypot(goal_x - current_x, goal_y - current_y)
                    
                    # 如果原目标点距离太远(>3米)或激光检测到前方有障碍，重新选择目标点
                    front_clear = True
                    if self.latest_distances is not None:
                        front_distances = self.latest_distances[8:13]  # 正前方
                        front_clear = min(front_distances) > 1.5  # 前方1.5米内无障碍
                    
                    if dist_to_approach_goal > 3.0 or not front_clear:
                        # 重新选择目标点：朝原目标方向前进1-2米
                        target_angle = math.atan2(goal_y - current_y, goal_x - current_x)
                        step = min(2.0, dist_to_approach_goal * 0.5)  # 步长为距离的一半，最大2米
                        new_goal_x = current_x + step * math.cos(target_angle)
                        new_goal_y = current_y + step * math.sin(target_angle)
                        self.lost_fire_goal = (new_goal_x, new_goal_y)
                        self.get_logger().info(f"火源丢失，智能调整目标点: 原目标({goal_x:.2f}, {goal_y:.2f})距离{dist_to_approach_goal:.2f}m，新目标({new_goal_x:.2f}, {new_goal_y:.2f})")
                    else:
                        self.lost_fire_goal = self.approach_goal
                        self.get_logger().info(f"火源丢失，继续前往靠近火源目标点({self.lost_fire_goal[0]:.2f}, {self.lost_fire_goal[1]:.2f})，距离{dist_to_approach_goal:.2f}m")
                    
                    self.goal_publisher.publish(Point(x=self.lost_fire_goal[0], y=self.lost_fire_goal[1], z=0.0))
                else:
                    self.lost_fire_goal = None
            # 记录进入搜索模式的时间（仅用于正常搜索模式）
            if self.search_mode_start_time is None:
                self.search_mode_start_time = time.time()
        elif self.search_mode is False:
            # 重新检测到火源，退出等待期
            self.lost_fire_waiting = False
            self.lost_fire_time = None
            self.lost_fire_goal = None
            self.search_mode_start_time = time.time()
            # 标记已进入靠近火源模式
            self.has_entered_approach_mode = True

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
    
    def temperature_callback(self, msg):
        self.current_temperature = msg.data

    def should_record_point(self, x, y):
        if self.last_recorded_point is None:
            return True
        lx, ly = self.last_recorded_point
        dist = math.hypot(x - lx, y - ly)
        return dist > 1.0

    def record_trajectory_point(self, x, y, temp):
        self.trajectory.append((x, y, temp))
        self.last_recorded_point = (x, y)
        with open(self.trajectory_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([x, y, temp])

    def start_recording(self):
        self.trajectory = []
        self.last_recorded_point = None
        self.recording = True
        # 写入表头
        with open(self.trajectory_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y', 'temperature'])

    def stop_recording(self):
        self.recording = False
        self.returning_path = self.trajectory[::-1]  # 反向路径

    def is_point_explored(self, x, y, threshold=1.0):
        """
        判断(x, y)点是否已在轨迹中被探索过，threshold为判定半径
        """
        for px, py, _ in self.trajectory:
            if math.hypot(x - px, y - py) < threshold:
                return True
        return False

    def control_loop(self):
        # 1. 记录起始点
        if not hasattr(self, 'start_position') and self.current_position is not None:
            self.start_position = (self.current_position.x, self.current_position.y)
            self.get_logger().info(f"记录起始点: {self.start_position}")

        # 轨迹记录逻辑
        if self.current_position is not None:
            if not self.recording:
                self.start_recording()
            x, y = self.current_position.x, self.current_position.y
            temp = self.current_temperature if self.current_temperature is not None else ''
            if self.should_record_point(x, y):
                self.record_trajectory_point(x, y, temp)

        # 2. 已到火源附近，静止
        if self.count_above_1000 > 100 and self.current_position is not None:
            if not hasattr(self, 'fire_position'):
                self.fire_position = (self.current_position.x, self.current_position.y)
                self.get_logger().info(f"到达火源附近，记录火源位置: {self.fire_position}")
                self.returning = True  # 新增：进入返回模式
                self.stop_recording()  # 停止记录轨迹
                self.goal_publisher.publish(Point(x=self.start_position[0], y=self.start_position[1], z=0.0))
                return
            
        # 5. 返回出发点
        if hasattr(self, 'returning') and self.returning and self.current_position is not None:
            # 优化返程：如果路径点大致在一条直线上，直接前往终点
            def are_points_colinear(points, tolerance=0.10):
                if len(points) < 4:  # 要求至少4个点才判断共线
                    return False
                # 轨迹点格式为(x, y, temp)，只取x, y坐标
                (x0, y0, _) = points[0]
                (x1, y1, _) = points[-1]
                dx = x1 - x0
                dy = y1 - y0
                norm = math.hypot(dx, dy)
                if norm < 0.5:  # 起终点距离太近也不判断共线
                    return False
                
                # 计算所有中间点到直线的距离
                max_deviation = 0.0
                for (x, y, _) in points[1:-1]:
                    # 点到直线距离公式
                    dist = abs(dy * x - dx * y + x1 * y0 - y1 * x0) / norm
                    max_deviation = max(max_deviation, dist)
                    if dist > tolerance:
                        return False
                
                # 额外检查：最大偏差不能超过路径总长度的5%
                path_length_threshold = norm * 0.05
                if max_deviation > path_length_threshold:
                    return False
                    
                return True

            if self.returning_path:
                # 检查返程路径是否大致共线
                check_n = min(5, len(self.returning_path))
                check_points = self.returning_path[:check_n]
                if are_points_colinear(check_points):
                    # 直接前往终点
                    final_point = self.returning_path[check_n-1]
                    self.get_logger().info(f"返程路径点大致共线，直接前往终点({final_point[0]:.2f}, {final_point[1]:.2f})")
                    self.goal_publisher.publish(Point(x=final_point[0], y=final_point[1], z=0.0))
                    # 如果到达终点则清空返程
                    dist_to_final = math.hypot(self.current_position.x - final_point[0], self.current_position.y - final_point[1])
                    if dist_to_final < 1.0:
                        self.returning_path = self.returning_path[check_n:]
                        if not self.returning_path:
                            self.goal_publisher.publish(Point(x=float('nan'), y=float('nan'), z=0.0))
                            self.get_logger().info(f"已返回出发点，起火点坐标为: {self.fire_position}")
                            print(f"起火点坐标: {self.fire_position}")
                            self.calculate_and_log_total_time()  # 输出总耗时
                            rclpy.shutdown()
                            return
                    return
                # 否则按原有逐点返程
                next_point = self.returning_path[0]
                dist_to_next = math.hypot(self.current_position.x - next_point[0], self.current_position.y - next_point[1])
                if not hasattr(self, 'return_point_set_time') or self.return_point_set_time is None:
                    self.return_point_set_time = time.time()
                if dist_to_next < 1.5:
                    self.returning_path.pop(0)
                    self.return_point_set_time = None
                    if not self.returning_path:
                        self.goal_publisher.publish(Point(x=float('nan'), y=float('nan'), z=0.0))
                        self.get_logger().info(f"已返回出发点，起火点坐标为: {self.fire_position}")
                        print(f"起火点坐标: {self.fire_position}")
                        self.calculate_and_log_total_time()  # 输出总耗时
                        rclpy.shutdown()
                        return
                elif time.time() - self.return_point_set_time > 20.0:
                    self.get_logger().warn(f"返回路径点({next_point[0]:.2f}, {next_point[1]:.2f})长时间无法到达，自动跳过！")
                    self.returning_path.pop(0)
                    self.return_point_set_time = None
                    return
                else:
                    self.goal_publisher.publish(Point(x=next_point[0], y=next_point[1], z=0.0))
            else:
                self.return_point_set_time = None
            return

        # 3. 搜索模式：自主探索
        if self.search_mode:
            now = time.time()
            # 优先处理丢失火源等待期（只在非返回模式下处理）
            if hasattr(self, 'lost_fire_waiting') and self.lost_fire_waiting and not (hasattr(self, 'returning') and self.returning):
                # 5秒内只发布丢失时的靠近火源目标点，不进入正常搜索
                if self.lost_fire_goal is not None:
                    self.goal_publisher.publish(Point(x=self.lost_fire_goal[0], y=self.lost_fire_goal[1], z=0.0))
                else:
                    self.goal_publisher.publish(Point(x=float('nan'), y=float('nan'), z=0.0))
                elapsed = now - self.lost_fire_time if self.lost_fire_time is not None else 0
                self.get_logger().info(f"火源丢失等待期: 已等待{elapsed:.1f}秒，目标点({self.lost_fire_goal[0]:.2f}, {self.lost_fire_goal[1]:.2f})" if self.lost_fire_goal else f"火源丢失等待期: 已等待{elapsed:.1f}秒，无目标点")
                if elapsed >= 5.0:
                    # 5秒后进入正常搜索
                    self.lost_fire_waiting = False
                    self.lost_fire_time = None
                    self.lost_fire_goal = None
                    self.search_mode_start_time = now  # 重新计时，进入正常搜索
                return
            # ---原有探索模式逻辑---
            if self.search_mode:
                now = time.time()
                if self.search_mode_start_time is not None:
                    elapsed = now - self.search_mode_start_time
                else:
                    elapsed = 0

                # 只有超过8秒才继续探索的逻辑修改
                # 如果曾经出现过count_above_1000 > 10，则需要等待8秒
                should_wait_8_seconds = self.has_count_above_10
                
                if should_wait_8_seconds and elapsed < 8:
                    self.target_x = None
                    self.target_y = None
                    self.current_goal = None
                    self.goal_set_time = None
                    self.pause_until = None
                    self.goal_publisher.publish(Point(x=float('nan'), y=float('nan'), z=0.0))
                    self.get_logger().info(f"探索模式已进入 {elapsed:.1f} 秒，未到8秒不探索 (曾出现count>10={self.has_count_above_10}, 当前count={self.count_above_1000})")
                    return

                # 探索目标点管理
                if not hasattr(self, 'current_goal'):
                    self.current_goal = None
                if not hasattr(self, 'goal_set_time'):
                    self.goal_set_time = None
                if not hasattr(self, 'pause_until'):
                    self.pause_until = None

                # 如果在暂停期，直接停住，等待0.5秒后再选新目标
                if self.pause_until is not None and now < self.pause_until:
                    self.goal_publisher.publish(Point(x=float('nan'), y=float('nan'), z=0.0))
                    self.get_logger().info(f"探索暂停中，等待雷达刷新...剩余{self.pause_until-now:.2f}s")
                    return
                elif self.pause_until is not None and now >= self.pause_until:
                    self.pause_until = None
                    self.current_goal = None
                    self.goal_set_time = None

                # 如果有目标点，且未超时，且未到达，则继续前进
                if self.current_goal is not None and self.goal_set_time is not None:
                    goal_x, goal_y = self.current_goal
                    dist_to_goal = math.hypot(self.current_position.x - goal_x, self.current_position.y - goal_y)
                    
                    # 判断是否为智能第一目标点（刚从靠近火源模式退出生成的）
                    is_smart_first_goal = hasattr(self, 'has_entered_approach_mode') and self.has_entered_approach_mode and not hasattr(self, 'switched_to_normal_exploration')
                    timeout_duration = 5.0 if is_smart_first_goal else 10.0  # 智能第一目标点5秒超时，其他10秒
                    
                    if dist_to_goal < 1.0:
                        # 到达目标点，清空目标，准备选新点
                        self.get_logger().info(f"已到达目标点({goal_x:.2f}, {goal_y:.2f})，准备选新点")
                        self.current_goal = None
                        self.goal_set_time = None
                        if is_smart_first_goal:
                            self.switched_to_normal_exploration = True  # 标记已切换到正常探索
                    elif now - self.goal_set_time < timeout_duration:
                        # 在超时时间内不切换目标，继续前进
                        self.goal_publisher.publish(Point(x=goal_x, y=goal_y, z=0.0))
                        return
                    else:
                        # 超时未到达，暂停0.5秒
                        timeout_type = "智能第一目标点5秒" if is_smart_first_goal else "普通目标点10秒"
                        self.get_logger().info(f"{timeout_type}未到达目标点({goal_x:.2f}, {goal_y:.2f})，暂停0.5秒后重新探索")
                        self.pause_until = now + 0.5
                        self.goal_publisher.publish(Point(x=float('nan'), y=float('nan'), z=0.0))
                        if is_smart_first_goal:
                            self.switched_to_normal_exploration = True  # 标记已切换到正常探索
                        return

                # 选新目标点（优先未探索区域）
                if self.latest_distances is not None and self.current_position is not None:
                    angle_step = math.pi / 20  # 180度/20
                    yaw = 0.0
                    q = self.current_orientation
                    if q is not None:
                        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0-2.0*(q.y*q.y + q.z*q.z))
                    
                    # 刚从靠近火源模式退出时，智能生成第一个探索目标点
                    if self.has_entered_approach_mode and self.need_smart_first_goal and len(self.trajectory) > 3:
                        self.get_logger().info(f"刚退出靠近火源模式，沿着已走过的路径生成第一个探索目标点")
                        
                        # 取最近的几个轨迹点，沿着路径方向继续前进
                        recent_points = self.trajectory[-4:]  # 取最近4个点
                        if len(recent_points) >= 2:
                            # 计算路径的延续方向
                            start_point = recent_points[0]  # 较早的点
                            end_point = recent_points[-1]   # 最新的点
                            
                            # 路径方向向量
                            path_dx = end_point[0] - start_point[0]
                            path_dy = end_point[1] - start_point[1]
                            path_length = math.hypot(path_dx, path_dy)
                            
                            if path_length > 0.1:  # 确保有足够的移动距离
                                # 归一化方向向量
                                path_direction_x = path_dx / path_length
                                path_direction_y = path_dy / path_length
                                
                                # 沿着路径方向前进3米
                                step = 3.0
                                goal_x = self.current_position.x + step * path_direction_x
                                goal_y = self.current_position.y + step * path_direction_y
                                
                                self.current_goal = (goal_x, goal_y)
                                self.goal_set_time = now
                                self.need_smart_first_goal = False  # 已生成智能目标点，后续使用距离优先
                                self.goal_publisher.publish(Point(x=goal_x, y=goal_y, z=0.0))
                                self.get_logger().info(f"探索模式: 沿路径方向前进，目标点({goal_x:.2f}, {goal_y:.2f})")
                                return
                    
                    # 原有的探索逻辑
                    candidates = []
                    for idx, dist in enumerate(self.latest_distances):
                        # 计算该方向的目标点
                        target_angle = yaw + (idx - 10) * angle_step
                        step = max(0.5, min(dist * 0.7, 5.0))
                        goal_x = self.current_position.x + step * math.cos(target_angle)
                        goal_y = self.current_position.y + step * math.sin(target_angle)
                        if not self.is_point_explored(goal_x, goal_y):
                            candidates.append((dist, idx, goal_x, goal_y))
                    if candidates:
                        # 选距离最大的未探索点
                        candidates.sort(reverse=True)  # 按距离降序
                        max_dist, max_idx, goal_x, goal_y = candidates[0]
                        self.current_goal = (goal_x, goal_y)
                        self.goal_set_time = now
                        self.last_goal = (goal_x, goal_y)
                        self.goal_publish_time = now
                        self.goal_publisher.publish(Point(x=goal_x, y=goal_y, z=0.0))
                        self.get_logger().info(f"探索模式: 优先未探索方向{max_idx}, 距离{max_dist:.2f}m, 发布新目标点({goal_x:.2f}, {goal_y:.2f})")
                    else:
                        # 所有方向都已探索，选距离最远的点（允许重复）
                        max_idx = int(np.argmax(self.latest_distances))
                        max_distance = self.latest_distances[max_idx]
                        target_angle = yaw + (max_idx - 10) * angle_step
                        step = max(0.3, min(max_distance * 0.7, 5.0))
                        goal_x = self.current_position.x + step * math.cos(target_angle)
                        goal_y = self.current_position.y + step * math.sin(target_angle)
                        self.current_goal = (goal_x, goal_y)
                        self.goal_set_time = now
                        self.last_goal = (goal_x, goal_y)
                        self.goal_publish_time = now
                        self.goal_publisher.publish(Point(x=goal_x, y=goal_y, z=0.0))
                        self.get_logger().info(f"探索模式: 所有方向已探索，选择距离最远方向{max_idx}, 距离{max_distance:.2f}m, 发布新目标点({goal_x:.2f}, {goal_y:.2f})")
                return

        # 4. 云台已指向火源方向，沿该方向靠近
        if not self.search_mode and self.current_position is not None:
            now = time.time()
            if not hasattr(self, 'approach_goal'):
                self.approach_goal = None
            if not hasattr(self, 'approach_goal_set_time'):
                self.approach_goal_set_time = None
            if not hasattr(self, 'approach_pause_until'):
                self.approach_pause_until = None

            # 暂停期，停住，等待0.5秒后再选新目标
            if self.approach_pause_until is not None and now < self.approach_pause_until:
                self.goal_publisher.publish(Point(x=float('nan'), y=float('nan'), z=0.0))
                self.get_logger().info(f"靠近火源暂停中，等待...剩余{self.approach_pause_until-now:.2f}s")
                return
            elif self.approach_pause_until is not None and now >= self.approach_pause_until:
                self.approach_pause_until = None
                self.approach_goal = None
                self.approach_goal_set_time = None

            # 如果有目标点，且未超时，且未到达，则继续前进
            if self.approach_goal is not None and self.approach_goal_set_time is not None:
                goal_x, goal_y = self.approach_goal
                dist_to_goal = math.hypot(self.current_position.x - goal_x, self.current_position.y - goal_y)
                if dist_to_goal < 0.5:
                    self.get_logger().info(f"已到达靠近火源目标点({goal_x:.2f}, {goal_y:.2f})，准备选新点")
                    self.approach_goal = None
                    self.approach_goal_set_time = None
                elif now - self.approach_goal_set_time < 10.0:
                    self.goal_publisher.publish(Point(x=goal_x, y=goal_y, z=0.0))
                    return
                else:
                    self.get_logger().info(f"10秒未到达靠近火源目标点({goal_x:.2f}, {goal_y:.2f})，暂停0.5秒后重新选点")
                    self.approach_pause_until = now + 0.5
                    self.goal_publisher.publish(Point(x=float('nan'), y=float('nan'), z=0.0))
                    return

            yaw = 0.0
            q = self.current_orientation
            if q is not None:
                yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0-2.0*(q.y*q.y + q.z*q.z))
            # 机器人前进方向 = 当前朝向 + 云台偏转角
            target_angle = yaw + self.target_horizontal_position * math.pi / 180.0
            
            # 记录当前靠近火源的大致方向
            self.last_approach_direction = target_angle

            # 结合激光雷达数据和云台指向，寻找最优前进方向
            best_idx = None
            max_dist = 0.0
            min_angle_diff = float('inf')
            found_valid = False
            
            # 检查云台角度是否稳定可靠
            servo_reliable = (self.servo_angle_stable_count >= 3 and 
                            abs(self.target_horizontal_position) != 0.0)  # 角度不为0
            
            if not servo_reliable:
                self.get_logger().warn(f"云台角度不稳定或接近0度({self.target_horizontal_position:.2f})，稳定计数: {self.servo_angle_stable_count}，等待可靠角度")
                # 云台角度不可靠时，停止前进，等待可靠的云台角度
                self.goal_publisher.publish(Point(x=float('nan'), y=float('nan'), z=0.0))
                return
            
            if self.latest_distances is not None:
                angle_step = math.pi / 20  # 21个区间，20个间隔
                
                # 首先检查云台指向30度范围内的所有距离
                target_area_distances = []
                for idx, dist in enumerate(self.latest_distances):
                    angle = (idx - 10) * angle_step  # idx=10为正前方
                    diff = abs(angle - self.target_horizontal_position * math.pi / 180.0)
                    
                    # 如果云台角度可靠，只考虑与云台指向夹角小于30度的方向
                    if diff < math.radians(30):
                        target_area_distances.append(dist)
                        if dist > 0.5:
                            found_valid = True
                            # 优先选距离最大的，距离一样大再选夹角最小的
                            if (dist > max_dist) or (abs(dist - max_dist) < 1e-3 and diff < min_angle_diff):
                                best_idx = idx
                                max_dist = dist
                                min_angle_diff = diff
                            print(f"方向 {idx} ({angle * 180 / math.pi:.1f}°): 距离={dist:.2f}, 夹角={diff * 180 / math.pi:.1f}°")

                # 检查是否需要绕行：云台指向30度范围内所有距离都小于1米
                need_detour = target_area_distances and all(d < 1.0 for d in target_area_distances)
                
                if need_detour:
                    self.get_logger().info(f"云台指向30度范围内距离都小于1米，启动绕行方案")
                    # 绕行方案：选择左右两侧距离较大的方向
                    left_distances = []  # 云台左侧
                    right_distances = []  # 云台右侧
                    
                    for idx, dist in enumerate(self.latest_distances):
                        angle = (idx - 10) * angle_step
                        relative_angle = angle - self.target_horizontal_position * math.pi / 180.0
                        
                        # 左侧：相对角度为正，且在合理范围内（30-90度）
                        if math.radians(30) <= relative_angle <= math.radians(90) and dist > 0.5:
                            left_distances.append((dist, idx, angle))
                        # 右侧：相对角度为负，且在合理范围内（-90到-30度）
                        elif math.radians(-90) <= relative_angle <= math.radians(-30) and dist > 0.5:
                            right_distances.append((dist, idx, angle))
                    
                    # 选择左右两侧中距离最大的方向
                    best_left = max(left_distances, key=lambda x: x[0]) if left_distances else None
                    best_right = max(right_distances, key=lambda x: x[0]) if right_distances else None
                    
                    if best_left and best_right:
                        # 都有选择，选择距离更大的
                        if best_left[0] > best_right[0]:
                            best_idx = best_left[1]
                            max_dist = best_left[0]
                            direction_info = f"绕行左侧，idx={best_idx}，距离={max_dist:.2f}m"
                        else:
                            best_idx = best_right[1]
                            max_dist = best_right[0]
                            direction_info = f"绕行右侧，idx={best_idx}，距离={max_dist:.2f}m"
                        found_valid = True
                    elif best_left:
                        best_idx = best_left[1]
                        max_dist = best_left[0]
                        direction_info = f"绕行左侧，idx={best_idx}，距离={max_dist:.2f}m"
                        found_valid = True
                    elif best_right:
                        best_idx = best_right[1]
                        max_dist = best_right[0]
                        direction_info = f"绕行右侧，idx={best_idx}，距离={max_dist:.2f}m"
                        found_valid = True
                    else:
                        # 无法绕行，后退
                        self.get_logger().warn("无法找到合适的绕行路径，执行后退")
                        step = 0.5
                        target_angle = yaw + math.pi  # 后退方向
                        goal_x = self.current_position.x + step * math.cos(target_angle)
                        goal_y = self.current_position.y + step * math.sin(target_angle)
                        self.approach_goal = (goal_x, goal_y)
                        self.approach_goal_set_time = now
                        self.goal_publisher.publish(Point(x=goal_x, y=goal_y, z=0.0))
                        self.get_logger().info(f"靠近火源: 后退避障，step={step:.2f}，目标点({goal_x:.2f}, {goal_y:.2f})")
                        return

                if found_valid and best_idx is not None:
                    # 选定方向
                    best_angle = (best_idx - 10) * angle_step
                    best_distance = self.latest_distances[best_idx]
                    # 步长为该方向距离的0.7，最小1.0米，最大5.0米
                    step = max(1.0, min(best_distance * 0.7, 5.0))
                    # 目标点的角度应为机器人朝向+最佳方向角
                    target_angle = yaw + best_angle
                    # 目标点坐标
                    goal_x = self.current_position.x + step * math.cos(target_angle)
                    goal_y = self.current_position.y + step * math.sin(target_angle)
                    
                    # 如果是绕行模式，使用之前设置的direction_info，否则使用默认信息
                    if not ('绕行' in locals() and direction_info):
                        direction_info = f"云台角度{self.target_horizontal_position:.2f}°可靠，选定方向idx={best_idx}"
                else:
                    # 没有找到合适方向，沿云台方向小步前进
                    step = 0.3
                    target_angle = yaw + self.target_horizontal_position * math.pi / 180.0
                    goal_x = self.current_position.x + step * math.cos(target_angle)
                    goal_y = self.current_position.y + step * math.sin(target_angle)
                    direction_info = f"云台角度{self.target_horizontal_position:.2f}°可靠但无合适激光方向，小步前进"
            else:
                # 没有激光数据时的处理
                # 云台可靠时，默认步长
                step = 0.5
                target_angle = yaw + self.target_horizontal_position * math.pi / 180.0
                goal_x = self.current_position.x + step * math.cos(target_angle)
                goal_y = self.current_position.y + step * math.sin(target_angle)
                direction_info = "无激光数据，云台角度可靠，使用云台指向"

            self.approach_goal = (goal_x, goal_y)
            self.approach_goal_set_time = now
            self.goal_publisher.publish(Point(x=goal_x, y=goal_y, z=0.0))
            self.get_logger().info(f"靠近火源: {direction_info}，step={step:.2f}，目标点({goal_x:.2f}, {goal_y:.2f})")
    
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
        node.calculate_and_log_total_time()  # 输出总耗时
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()