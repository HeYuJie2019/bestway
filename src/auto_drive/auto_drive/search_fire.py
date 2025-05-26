#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String, Bool, Int32
from geometry_msgs.msg import Twist
import math
import time  # 添加时间模块
import pyzed.sl as sl  # 导入 ZED SDK
import numpy as np
from nav_msgs.msg import Odometry

class AutoDriveNode(Node):
    def __init__(self):
        super().__init__('search_fire_node')

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
        # 获取前方距离（使用 ZED 深度相机）
        self.front_zed = self.get_front_distance()

        if self.latest_distances is None:
            return  # 如果没有激光雷达数据，什么都不做
        if self.front_zed is None:
            return  # 如果没有深度相机数据，什么都不做
        """
        主控制循环：
        1. 记录起点
        2. 搜寻火源（云台search_mode控制）
        3. 发现火源后向目标移动，途中丢失目标则重新搜寻
        4. 遇到障碍物时综合目标方向和已搜寻点避障
        5. 找到火源后记录坐标，继续搜寻下一个
        6. 全部搜寻完毕后返回起点
        """
        # 1. 记录起点
        if not hasattr(self, 'start_position') and self.odom_position is not None:
            self.start_position = (self.odom_position.x, self.odom_position.y)
            self.visited_fire_points = []  # 已搜寻火源点
            self.visited_obstacles = []    # 已遇到障碍物点
            self.searching_finished = False
            self.get_logger().info(f"记录起点: {self.start_position}")

        # 2. 搜寻是否结束
        if getattr(self, 'searching_finished', False):
            # 返回起点
            if self.odom_position is not None:
                dx = self.start_position[0] - self.odom_position.x
                dy = self.start_position[1] - self.odom_position.y
                distance_to_start = math.hypot(dx, dy)
                self.get_logger().info(f"返回起点，距离: {distance_to_start:.2f} 米")
                if distance_to_start < 0.3:
                    self.stop()
                    self.get_logger().info("已返回起点，任务完成")
                    rclpy.shutdown()
                    return
                else:
                    # 简单导航回起点（可替换为更复杂的路径规划）
                    angle_to_start = math.atan2(dy, dx)
                    # 这里可根据实际情况调整导航策略
                    self.drive_forward(self.speed * 0.5)
            return

        # 3. 搜寻火源主流程
        if self.search_mode:
            self.stop()
            # 检查是否等待超过10秒
            if self.search_mode_start_time is not None:
                wait_time = time.time() - self.search_mode_start_time
                if wait_time > 5:
                    self.get_logger().info("搜寻超时，主动前往未探索区域辅助云台寻找...")

                    # 维护已探索点列表
                    if not hasattr(self, 'explored_points'):
                        self.explored_points = []

                    # 记录当前位置为已探索
                    if self.odom_position is not None:
                        current_point = (round(self.odom_position.x, 1), round(self.odom_position.y, 1))
                        if current_point not in self.explored_points:
                            self.explored_points.append(current_point)

                    # 生成一圈候选目标点（以当前位置为中心，间隔1米，8个方向）
                    if self.odom_position is not None:
                        current_x = self.odom_position.x
                        current_y = self.odom_position.y
                        candidate_points = []
                        for angle in np.linspace(0, 2*math.pi, 8, endpoint=False):
                            target_x = current_x + 1.0 * math.cos(angle)
                            target_y = current_y + 1.0 * math.sin(angle)
                            candidate_points.append((round(target_x, 1), round(target_y, 1)))

                        # 选择距离所有已探索点最远的点
                        max_min_dist = -1
                        best_point = None
                        for pt in candidate_points:
                            min_dist = min([math.hypot(pt[0]-ep[0], pt[1]-ep[1]) for ep in self.explored_points] or [float('inf')])
                            if min_dist > max_min_dist and pt not in self.explored_points:
                                max_min_dist = min_dist
                                best_point = pt

                        if best_point is not None:
                            # 计算目标点方向
                            dx = best_point[0] - current_x
                            dy = best_point[1] - current_y
                            angle_to_target = math.atan2(dy, dx)
                            self.get_logger().info(f"主动探索新区域，目标点: {best_point}，方向角: {math.degrees(angle_to_target):.1f}°")

                            # 简单转向（假设机器人朝向为x正方向，可根据实际情况调整）
                            turn_direction = 1 if angle_to_target > 0 else -1
                            turn_time = abs(angle_to_target) / 1.5  # 1.5为角速度，需与turn_left/turn_right一致
                            t_start = time.time()
                            while time.time() - t_start < turn_time:
                                if turn_direction > 0:
                                    self.turn_left()
                                else:
                                    self.turn_right()
                                rclpy.spin_once(self, timeout_sec=0.05)
                            self.stop()

                            # 前进到目标点，实时判断距离
                            self.get_logger().info(f"前往目标点: {best_point}")
                            while True:
                                if self.odom_position is None:
                                    break
                                cur_pos = (self.odom_position.x, self.odom_position.y)
                                dist = math.hypot(best_point[0] - cur_pos[0], best_point[1] - cur_pos[1])
                                if dist < 0.2:  # 到达目标点
                                    self.stop()
                                    self.get_logger().info(f"已到达目标点: {best_point}")
                                    break
                                # 若前方有障碍物则中断前进
                                if self.get_front_distance() < self.safe_distance:
                                    self.stop()
                                    self.get_logger().info("前往新区域途中遇到障碍物，停止前进")
                                    break
                                self.drive_forward(self.base_speed * 0.5)
                                rclpy.spin_once(self, timeout_sec=0.05)
                            self.explored_points.append(best_point)

                    self.search_mode_start_time = time.time()  # 重置等待时间
                    return
            else:
                self.search_mode_start_time = time.time()
            self.get_logger().info("云台正在搜寻火源，机器人等待...")
            return

        # 4. 云台已锁定火源，准备前进
        if not self.search_mode:
            # 检查是否已到达火源点
            if self.count_above_1000 > 3000:
                # 记录火源点
                if self.odom_position is not None:
                    fire_point = (self.odom_position.x, self.odom_position.y)
                    # 判断是否为新火源点
                    if not any(math.hypot(fire_point[0]-p[0], fire_point[1]-p[1]) < 0.5 for p in self.visited_fire_points):
                        self.visited_fire_points.append(fire_point)
                        self.get_logger().info(f"发现新火源点并记录: {fire_point}")
                    else:
                        self.get_logger().info("火源点已记录，跳过")
                # 判断是否搜寻完毕（这里假设搜寻3个火源点为全部搜寻完毕，可根据实际需求调整）
                if len(self.visited_fire_points) >= 1:
                    self.searching_finished = True
                    self.get_logger().info("所有火源点已搜寻完毕，准备返回起点")
                else:
                    # 继续搜寻下一个火源点，等待云台重新进入search_mode
                    self.get_logger().info("继续搜寻下一个火源点")
                return

            # 5. 前进到火源方向，途中丢失目标则等待云台重新搜寻
            self.front_zed = self.get_front_distance()
            self.calculate_dynamic_speed_and_distance(self.front_zed)

            # 检查障碍物
            if self.front_zed < self.safe_distance:
                # 记录障碍物点
                if self.odom_position is not None:
                    obs_point = (self.odom_position.x, self.odom_position.y)
                    if not any(math.hypot(obs_point[0]-p[0], obs_point[1]-p[1]) < 0.3 for p in self.visited_obstacles):
                        self.visited_obstacles.append(obs_point)
                        self.get_logger().info(f"记录障碍物点: {obs_point}")
                
                def turn_until_clear(turn_func, direction_str):
                    self.get_logger().info(f"{direction_str}，持续转向直到前方无障碍")
                    while self.get_front_distance() < self.safe_distance:
                        turn_func()
                        rclpy.spin_once(self, timeout_sec=0.05)
                    self.stop()
                    self.get_logger().info("前方已无障碍，准备前进")
                    self.drive_forward(self.speed * 0.5)
                    time.sleep(0.8)  # 前进一小段，确保绕开障碍物
                    self.stop()

                # 综合目标方向和已避障点选择转向
                # 优先考虑远离已避障点的方向，其次再考虑目标方向
                if self.visited_obstacles:
                    left_score = sum([1/math.hypot(self.odom_position.x-p[0], self.odom_position.y-p[1]) 
                                    for p in self.visited_obstacles if self.odom_position.x-p[0]<0]) + 1e-3
                    right_score = sum([1/math.hypot(self.odom_position.x-p[0], self.odom_position.y-p[1]) 
                                    for p in self.visited_obstacles if self.odom_position.x-p[0]>0]) + 1e-3
                    if left_score < right_score:
                        turn_until_clear(self.turn_right, "优先远离左侧已避障点，右转")
                    else:
                        turn_until_clear(self.turn_left, "优先远离右侧已避障点，左转")
                elif self.target_horizontal_position > 20:
                    turn_until_clear(self.turn_left, "障碍物前方，目标偏左，左转")
                elif self.target_horizontal_position < -20:
                    turn_until_clear(self.turn_right, "障碍物前方，目标偏右，右转")
                else:
                    turn_until_clear(self.turn_left, "障碍物前方，方向不明确，默认左转")
                return

            if abs(self.target_horizontal_position) > 20 :
                # 目标有明显偏向，先调整方向
                turn_time = abs(self.target_horizontal_position) * 0.01
                t_start = time.time()
                if self.target_horizontal_position > 0:
                    self.get_logger().info(f"向左调整方向，时间: {turn_time:.2f}s")
                    while time.time() - t_start < turn_time:
                        self.turn_left()
                        rclpy.spin_once(self, timeout_sec=0.05)
                else:
                    self.get_logger().info(f"向右调整方向，时间: {turn_time:.2f}s")
                    while time.time() - t_start < turn_time:
                        self.turn_right()
                        rclpy.spin_once(self, timeout_sec=0.05)
            else:
                # 目标基本正前方，直接前进
                self.drive_forward(self.speed)
        
    
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