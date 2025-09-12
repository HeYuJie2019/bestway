#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import Header
import numpy as np
import math
from collections import deque
from typing import Tuple, List, Optional
import time
import heapq


class SimpleNavigationController(Node):
    """
    基于二维地图的简单导航控制器
    根据目标点和当前位姿进行路径规划和控制
    """
    
    def __init__(self):
        super().__init__('navigation_controller')
        
        # 声明参数
        self.declare_parameter('goal_tolerance', 0.5)    # 目标点容忍度，稍微增大
        self.declare_parameter('obstacle_threshold', 70) # 障碍物阈值
        self.declare_parameter('control_frequency', 10.0) # 控制频率
        self.declare_parameter('obstacle_inflation_radius', 0.3) # 障碍物膨胀半径
        self.declare_parameter('max_planning_time', 5.0) # 最大规划时间
        self.declare_parameter('stuck_detection_time', 3.0) # 卡住检测时间
        self.declare_parameter('stuck_distance_threshold', 0.1) # 卡住距离阈值
        self.declare_parameter('path_smoothing_factor', 0.5) # 路径平滑因子
        self.declare_parameter('waypoint_spacing', 0.5) # 路径点间距
        # 添加临时目标相关参数
        self.declare_parameter('temp_goal_distance', 1.5) # 临时目标距离
        self.declare_parameter('temp_goal_update_frequency', 2.0) # 临时目标更新频率
        self.declare_parameter('lookahead_distance', 2.0) # 前瞻距离
        self.declare_parameter('obstacle_avoidance_distance', 1.0) # 避障距离
        self.declare_parameter('min_passage_width', 0.6) # 最小通道宽度
        self.declare_parameter('max_temp_goal_distance', 3.0) # 最大临时目标距离
        self.declare_parameter('min_temp_goal_distance', 0.5) # 最小临时目标距离
        self.declare_parameter('robot_movement_threshold', 0.2) # 机器人移动检测阈值
        
        # 获取参数
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.control_frequency = self.get_parameter('control_frequency').value
        self.obstacle_inflation_radius = self.get_parameter('obstacle_inflation_radius').value
        self.max_planning_time = self.get_parameter('max_planning_time').value
        self.stuck_detection_time = self.get_parameter('stuck_detection_time').value
        self.stuck_distance_threshold = self.get_parameter('stuck_distance_threshold').value
        self.path_smoothing_factor = self.get_parameter('path_smoothing_factor').value
        self.waypoint_spacing = self.get_parameter('waypoint_spacing').value
        self.temp_goal_distance = self.get_parameter('temp_goal_distance').value
        self.temp_goal_update_frequency = self.get_parameter('temp_goal_update_frequency').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.obstacle_avoidance_distance = self.get_parameter('obstacle_avoidance_distance').value
        self.min_passage_width = self.get_parameter('min_passage_width').value
        self.max_temp_goal_distance = self.get_parameter('max_temp_goal_distance').value
        self.min_temp_goal_distance = self.get_parameter('min_temp_goal_distance').value
        self.robot_movement_threshold = self.get_parameter('robot_movement_threshold').value
        
        # 状态变量
        self.current_pose = None
        self.goal_pose = None
        self.occupancy_grid = None
        self.is_navigating = False
        self.path = []
        self.current_path_index = 0
        
        # 控制状态
        self.control_state = "IDLE"  # IDLE, NAVIGATING, REACHED, STUCK
        self.last_control_time = time.time()
        self.last_temp_goal_time = time.time()  # 上次发布临时目标的时间
        self.current_temp_goal = None  # 当前临时目标
        
        # 机器人移动检测
        self.last_position = None
        self.last_position_check_time = time.time()
        self.temp_goal_retry_count = 0  # 临时目标重试次数
        self.max_temp_goal_retries = 3  # 最大重试次数
        
        # 卡住检测
        self.position_history = deque(maxlen=int(self.stuck_detection_time * self.control_frequency))
        self.last_replan_time = 0.0
        self.replan_cooldown = 2.0  # 重新规划冷却时间
        
        # 地图处理
        self.inflated_map = None
        
        # 创建订阅者
        self.goal_subscriber = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/Odometry',
            self.odom_callback,
            10
        )
        
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            1
        )
        
        # 创建发布者
        self.temp_goal_publisher = self.create_publisher(
            Point,
            '/temp_goal_pose',
            10
        )
        
        # 创建路径发布者用于可视化
        self.path_publisher = self.create_publisher(
            Path,
            '/path',
            10
        )
        
        # 创建机器人轨迹发布者
        self.trajectory_publisher = self.create_publisher(
            Path,
            '/robot_trajectory',
            10
        )
        
        # 机器人轨迹
        self.robot_trajectory = Path()
        self.robot_trajectory.header.frame_id = "map"
        
        # 创建定时器
        timer_period = 1.0 / self.temp_goal_update_frequency
        self.control_timer = self.create_timer(timer_period, self.control_loop)
        
        self.get_logger().info('Simple Navigation Controller started - Enhanced Version')
        self.get_logger().info(f'Goal tolerance: {self.goal_tolerance} m')
        self.get_logger().info(f'Min passage width: {self.min_passage_width} m')
        self.get_logger().info(f'Temp goal distance range: {self.min_temp_goal_distance}-{self.max_temp_goal_distance} m')
        self.get_logger().info(f'Temp goal update frequency: {self.temp_goal_update_frequency} Hz')
        self.get_logger().info(f'Lookahead distance: {self.lookahead_distance} m')
        self.get_logger().info(f'Obstacle avoidance distance: {self.obstacle_avoidance_distance} m')
        self.get_logger().info(f'Robot movement threshold: {self.robot_movement_threshold} m')
        
    def goal_callback(self, msg: PoseStamped):
        """目标点回调函数"""
        self.goal_pose = msg.pose
        self.is_navigating = True
        self.current_path_index = 0
        self.control_state = "NAVIGATING"
        self.current_temp_goal = None  # 重置临时目标
        
        goal_x = self.goal_pose.position.x
        goal_y = self.goal_pose.position.y
        
        self.get_logger().info(f'New goal received: ({goal_x:.2f}, {goal_y:.2f})')
        
        # 简单路径规划
        self.plan_path()
        
    def odom_callback(self, msg: Odometry):
        """里程计回调函数"""
        self.current_pose = msg.pose.pose
        
        # 更新机器人轨迹
        self.update_robot_trajectory()
        
    def update_robot_trajectory(self):
        """更新机器人轨迹并发布"""
        if self.current_pose is None:
            return
        
        # 创建新的轨迹点
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "map"
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = self.current_pose
        
        # 添加到轨迹中（限制轨迹长度避免内存问题）
        self.robot_trajectory.poses.append(pose_stamped)
        if len(self.robot_trajectory.poses) > 1000:  # 保持最近1000个点
            self.robot_trajectory.poses.pop(0)
        
        # 更新轨迹头部时间戳
        self.robot_trajectory.header.stamp = pose_stamped.header.stamp
        
        # 发布轨迹
        self.trajectory_publisher.publish(self.robot_trajectory)
        
    def map_callback(self, msg: OccupancyGrid):
        """地图回调函数"""
        self.occupancy_grid = msg
        self.inflated_map = self.inflate_obstacles(msg)
    
    def inflate_obstacles(self, occupancy_grid: OccupancyGrid) -> np.ndarray:
        """膨胀障碍物以提供安全距离"""
        if occupancy_grid is None:
            return None
            
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        resolution = occupancy_grid.info.resolution
        
        # 将1D数据转换为2D数组
        grid_data = np.array(occupancy_grid.data).reshape((height, width))
        
        # 计算膨胀半径（像素数）
        inflation_pixels = int(self.obstacle_inflation_radius / resolution)
        
        # 创建膨胀后的地图
        inflated = np.copy(grid_data)
        
        # 找到所有障碍物位置
        obstacle_positions = np.where(grid_data >= self.obstacle_threshold)
        
        # 对每个障碍物进行膨胀
        for y, x in zip(obstacle_positions[0], obstacle_positions[1]):
            for dy in range(-inflation_pixels, inflation_pixels + 1):
                for dx in range(-inflation_pixels, inflation_pixels + 1):
                    if dx*dx + dy*dy <= inflation_pixels*inflation_pixels:
                        ny, nx = y + dy, x + dx
                        if 0 <= ny < height and 0 <= nx < width:
                            if grid_data[ny, nx] != -1:  # 不修改未知区域
                                inflated[ny, nx] = max(inflated[ny, nx], self.obstacle_threshold)
        
        return inflated
        
    def quaternion_to_yaw(self, quat: Quaternion) -> float:
        """四元数转偏航角"""
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def normalize_angle(self, angle: float) -> float:
        """角度归一化到[-π, π]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def distance_to_goal(self) -> float:
        """计算到目标点的距离"""
        if self.current_pose is None or self.goal_pose is None:
            return float('inf')
        
        dx = self.goal_pose.position.x - self.current_pose.position.x
        dy = self.goal_pose.position.y - self.current_pose.position.y
        return math.sqrt(dx*dx + dy*dy)
    
    def is_path_clear(self, start_x: float, start_y: float, end_x: float, end_y: float) -> bool:
        """检查路径是否无障碍物（使用膨胀后的地图）"""
        if self.inflated_map is None:
            return True
        
        # 使用Bresenham直线算法检查路径
        resolution = self.occupancy_grid.info.resolution
        origin_x = self.occupancy_grid.info.origin.position.x
        origin_y = self.occupancy_grid.info.origin.position.y
        width = self.occupancy_grid.info.width
        height = self.occupancy_grid.info.height
        
        # 转换为栅格坐标
        start_grid_x = int((start_x - origin_x) / resolution)
        start_grid_y = int((start_y - origin_y) / resolution)
        end_grid_x = int((end_x - origin_x) / resolution)
        end_grid_y = int((end_y - origin_y) / resolution)
        
        # Bresenham直线算法
        dx = abs(end_grid_x - start_grid_x)
        dy = abs(end_grid_y - start_grid_y)
        x = start_grid_x
        y = start_grid_y
        
        x_inc = 1 if end_grid_x > start_grid_x else -1
        y_inc = 1 if end_grid_y > start_grid_y else -1
        error = dx - dy
        
        dx *= 2
        dy *= 2
        
        while True:
            # 检查当前点是否在地图范围内
            if 0 <= x < width and 0 <= y < height:
                cell_value = self.inflated_map[y, x]
                if cell_value >= self.obstacle_threshold:  # 障碍物
                    return False
            else:
                # 超出地图边界视为障碍物
                return False
            
            if x == end_grid_x and y == end_grid_y:
                break
                
            if error > 0:
                x += x_inc
                error -= dy
            else:
                y += y_inc
                error += dx
        
        return True
    
    def plan_path(self):
        """使用A*算法进行路径规划"""
        if self.current_pose is None or self.goal_pose is None or self.occupancy_grid is None:
            return
        
        start_time = time.time()
        self.path = []
        
        start_x = self.current_pose.position.x
        start_y = self.current_pose.position.y
        goal_x = self.goal_pose.position.x
        goal_y = self.goal_pose.position.y
        
        # 使用A*算法规划路径
        path_points = self.astar_planning(start_x, start_y, goal_x, goal_y)
        
        if path_points:
            # 平滑路径
            self.path = self.smooth_path(path_points)
            self.get_logger().info(f'Path planned with {len(self.path)} waypoints in {time.time() - start_time:.3f}s')
            
            # 发布路径用于可视化
            self.publish_path_for_visualization()
        else:
            self.get_logger().warn('Failed to find a path to the goal')
            # 如果A*失败，尝试简单的直线路径
            if self.is_path_clear(start_x, start_y, goal_x, goal_y):
                self.path = [(goal_x, goal_y)]
                self.publish_path_for_visualization()
    
    def astar_planning(self, start_x: float, start_y: float, goal_x: float, goal_y: float) -> List[Tuple[float, float]]:
        """A*路径规划算法"""
        if self.inflated_map is None:
            return []
        
        resolution = self.occupancy_grid.info.resolution
        origin_x = self.occupancy_grid.info.origin.position.x
        origin_y = self.occupancy_grid.info.origin.position.y
        width = self.occupancy_grid.info.width
        height = self.occupancy_grid.info.height
        
        # 转换为栅格坐标
        start_grid_x = int((start_x - origin_x) / resolution)
        start_grid_y = int((start_y - origin_y) / resolution)
        goal_grid_x = int((goal_x - origin_x) / resolution)
        goal_grid_y = int((goal_y - origin_y) / resolution)
        
        # 检查起点和终点是否在地图范围内
        if not (0 <= start_grid_x < width and 0 <= start_grid_y < height):
            self.get_logger().warn(f'Start point ({start_x}, {start_y}) is outside map bounds')
            return []
        
        if not (0 <= goal_grid_x < width and 0 <= goal_grid_y < height):
            self.get_logger().warn(f'Goal point ({goal_x}, {goal_y}) is outside map bounds')
            return []
        
        # 检查起点和终点是否可通行
        if self.inflated_map[start_grid_y, start_grid_x] >= self.obstacle_threshold:
            self.get_logger().warn(f'Start point is in obstacle')
            return []
        
        if self.inflated_map[goal_grid_y, goal_grid_x] >= self.obstacle_threshold:
            self.get_logger().warn(f'Goal point is in obstacle')
            return []
        
        # A*算法
        open_set = []
        heapq.heappush(open_set, (0, start_grid_x, start_grid_y))
        
        came_from = {}
        g_score = {(start_grid_x, start_grid_y): 0}
        f_score = {(start_grid_x, start_grid_y): self.heuristic(start_grid_x, start_grid_y, goal_grid_x, goal_grid_y)}
        
        # 8方向移动
        directions = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
        
        start_planning_time = time.time()
        
        while open_set:
            # 检查规划时间限制
            if time.time() - start_planning_time > self.max_planning_time:
                self.get_logger().warn('A* planning timeout')
                break
            
            current_f, current_x, current_y = heapq.heappop(open_set)
            
            # 到达目标
            if current_x == goal_grid_x and current_y == goal_grid_y:
                path = []
                x, y = current_x, current_y
                while (x, y) in came_from:
                    # 转换回世界坐标
                    world_x = x * resolution + origin_x
                    world_y = y * resolution + origin_y
                    path.append((world_x, world_y))
                    x, y = came_from[(x, y)]
                
                # 添加起点
                path.append((start_x, start_y))
                path.reverse()
                return path
            
            # 探索邻居
            for dx, dy in directions:
                neighbor_x = current_x + dx
                neighbor_y = current_y + dy
                
                # 检查边界
                if not (0 <= neighbor_x < width and 0 <= neighbor_y < height):
                    continue
                
                # 检查障碍物
                if self.inflated_map[neighbor_y, neighbor_x] >= self.obstacle_threshold:
                    continue
                
                # 计算移动代价
                move_cost = math.sqrt(dx*dx + dy*dy)
                tentative_g_score = g_score[(current_x, current_y)] + move_cost
                
                if (neighbor_x, neighbor_y) not in g_score or tentative_g_score < g_score[(neighbor_x, neighbor_y)]:
                    came_from[(neighbor_x, neighbor_y)] = (current_x, current_y)
                    g_score[(neighbor_x, neighbor_y)] = tentative_g_score
                    f_score[(neighbor_x, neighbor_y)] = tentative_g_score + self.heuristic(neighbor_x, neighbor_y, goal_grid_x, goal_grid_y)
                    heapq.heappush(open_set, (f_score[(neighbor_x, neighbor_y)], neighbor_x, neighbor_y))
        
        return []
    
    def heuristic(self, x1: int, y1: int, x2: int, y2: int) -> float:
        """A*算法的启发式函数（欧几里得距离）"""
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
    
    def smooth_path(self, path_points: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """高级路径平滑，创建更丝滑的路径"""
        if len(path_points) <= 2:
            return path_points
        
        # 第一步：移除不必要的中间点
        simplified_path = self.simplify_path(path_points)
        
        # 第二步：插值生成更多中间点
        dense_path = self.interpolate_path(simplified_path)
        
        # 第三步：应用移动平均平滑
        smoothed_path = self.apply_smoothing_filter(dense_path)
        
        return smoothed_path
    
    def simplify_path(self, path_points: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """简化路径，移除不必要的中间点"""
        if len(path_points) <= 2:
            return path_points
        
        simplified_path = [path_points[0]]  # 添加起点
        
        i = 0
        while i < len(path_points) - 1:
            # 尝试直接连接到最远的可达点
            j = len(path_points) - 1
            while j > i + 1:
                if self.is_path_clear(path_points[i][0], path_points[i][1], 
                                    path_points[j][0], path_points[j][1]):
                    simplified_path.append(path_points[j])
                    i = j
                    break
                j -= 1
            else:
                # 如果无法跳跃，移动到下一个点
                i += 1
                if i < len(path_points):
                    simplified_path.append(path_points[i])
        
        return simplified_path
    
    def interpolate_path(self, path_points: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """在路径点之间插值，创建更密集的路径点"""
        if len(path_points) <= 1:
            return path_points
        
        interpolated_path = [path_points[0]]
        
        for i in range(len(path_points) - 1):
            current_point = path_points[i]
            next_point = path_points[i + 1]
            
            # 计算两点间距离
            distance = math.sqrt(
                (next_point[0] - current_point[0])**2 + 
                (next_point[1] - current_point[1])**2
            )
            
            # 根据距离确定插值点数量
            num_interpolations = max(1, int(distance / self.waypoint_spacing))
            
            # 添加插值点
            for j in range(1, num_interpolations + 1):
                t = j / num_interpolations
                interpolated_x = current_point[0] + t * (next_point[0] - current_point[0])
                interpolated_y = current_point[1] + t * (next_point[1] - current_point[1])
                
                # 确保插值点不在障碍物中
                if self.is_path_clear(current_point[0], current_point[1], interpolated_x, interpolated_y):
                    interpolated_path.append((interpolated_x, interpolated_y))
                else:
                    # 如果插值点不安全，直接连接到下一个点
                    break
        
        return interpolated_path
    
    def apply_smoothing_filter(self, path_points: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """应用移动平均滤波器平滑路径"""
        if len(path_points) <= 2:
            return path_points
        
        smoothed_path = []
        window_size = 3  # 滑动窗口大小
        
        for i in range(len(path_points)):
            # 计算窗口范围
            start_idx = max(0, i - window_size // 2)
            end_idx = min(len(path_points), i + window_size // 2 + 1)
            
            # 计算窗口内点的加权平均
            sum_x, sum_y = 0.0, 0.0
            weight_sum = 0.0
            
            for j in range(start_idx, end_idx):
                # 距离中心点越近，权重越大
                distance_to_center = abs(j - i)
                weight = 1.0 / (1.0 + distance_to_center * self.path_smoothing_factor)
                
                sum_x += path_points[j][0] * weight
                sum_y += path_points[j][1] * weight
                weight_sum += weight
            
            # 计算加权平均位置
            if weight_sum > 0:
                smoothed_x = sum_x / weight_sum
                smoothed_y = sum_y / weight_sum
                
                # 确保平滑后的点仍然可达
                if i == 0 or self.is_path_clear(smoothed_path[-1][0], smoothed_path[-1][1], 
                                              smoothed_x, smoothed_y):
                    smoothed_path.append((smoothed_x, smoothed_y))
                else:
                    # 如果平滑后的点不可达，使用原始点
                    smoothed_path.append(path_points[i])
            else:
                smoothed_path.append(path_points[i])
        
        return smoothed_path
    
    def publish_path_for_visualization(self):
        """发布路径用于在RViz中可视化"""
        if not self.path:
            return
        
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for x, y in self.path:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.header.stamp = path_msg.header.stamp
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.w = 1.0
            path_msg.poses.append(pose_stamped)
        
        self.path_publisher.publish(path_msg)
    
    def detect_stuck(self) -> bool:
        """检测机器人是否卡住"""
        if self.current_pose is None:
            return False
        
        current_pos = (self.current_pose.position.x, self.current_pose.position.y)
        self.position_history.append(current_pos)
        
        if len(self.position_history) < int(self.stuck_detection_time * self.control_frequency * 0.8):
            return False
        
        # 计算位置变化
        start_pos = self.position_history[0]
        end_pos = self.position_history[-1]
        total_distance = math.sqrt((end_pos[0] - start_pos[0])**2 + (end_pos[1] - start_pos[1])**2)
        
        return total_distance < self.stuck_distance_threshold
    
    def get_lookahead_point_on_path(self) -> Optional[Tuple[float, float]]:
        """在路径上找到前瞻点"""
        if not self.path or self.current_pose is None:
            return None
        
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        
        # 从当前路径索引开始寻找前瞻点
        for i in range(self.current_path_index, len(self.path)):
            path_x, path_y = self.path[i]
            distance = math.sqrt((path_x - current_x)**2 + (path_y - current_y)**2)
            
            # 找到合适距离的点
            if distance >= self.lookahead_distance:
                return (path_x, path_y)
        
        # 如果没有找到，返回路径终点
        if self.path:
            return self.path[-1]
        
        return None
    
    def check_temp_goal_safety(self, temp_x: float, temp_y: float) -> bool:
        """检查临时目标点是否安全 - 实用版本"""
        if self.current_pose is None:
            return False
        
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        
        # 使用基本的安全边距检查临时目标点
        basic_safety_margin = self.obstacle_avoidance_distance * 0.9  # 进一步减少安全边距
        
        # 检查临时目标点本身是否安全
        if not self.is_point_safe_with_margin(temp_x, temp_y, basic_safety_margin):
            return False
        
        # 简化路径检查：只检查到临时目标80%的距离
        distance_to_temp = math.sqrt((temp_x - current_x)**2 + (temp_y - current_y)**2)
        if distance_to_temp > 1.5:  # 只对较远的目标进行路径检查
            path_safety_margin = self.obstacle_avoidance_distance * 0.7  # 进一步减少路径安全边距
            check_distance = distance_to_temp * 0.8  # 只检查80%的路径
            
            # 计算检查点
            direction_x = (temp_x - current_x) / distance_to_temp
            direction_y = (temp_y - current_y) / distance_to_temp
            check_x = current_x + direction_x * check_distance
            check_y = current_y + direction_y * check_distance
            
            if not self.is_point_safe_with_margin(check_x, check_y, path_safety_margin):
                return False
        
        return True
    
    def is_point_safe_with_margin(self, x: float, y: float, safety_margin: float) -> bool:
        """检查点是否安全（带安全边距）- 实用版本"""
        if self.inflated_map is None or self.occupancy_grid is None:
            return True
        
        resolution = self.occupancy_grid.info.resolution
        origin_x = self.occupancy_grid.info.origin.position.x
        origin_y = self.occupancy_grid.info.origin.position.y
        width = self.occupancy_grid.info.width
        height = self.occupancy_grid.info.height
        
        # 转换为栅格坐标
        center_grid_x = int((x - origin_x) / resolution)
        center_grid_y = int((y - origin_y) / resolution)
        
        # 计算安全边距对应的栅格数 - 使用基本的安全区域
        base_safety_pixels = int(safety_margin / resolution)
        
        # 检查以目标点为中心的圆形区域
        for dy in range(-base_safety_pixels, base_safety_pixels + 1):
            for dx in range(-base_safety_pixels, base_safety_pixels + 1):
                # 计算距离
                distance = math.sqrt(dx*dx + dy*dy) * resolution
                if distance > safety_margin:  # 移除额外缓冲
                    continue
                    
                check_x = center_grid_x + dx
                check_y = center_grid_y + dy
                
                # 检查边界
                if not (0 <= check_x < width and 0 <= check_y < height):
                    return False
                
                # 检查是否在障碍物中 - 使用标准阈值
                if self.inflated_map[check_y, check_x] >= self.obstacle_threshold:
                    return False
        
        return True
    
    def get_min_distance_to_obstacle(self, x: float, y: float) -> float:
        """获取点到最近障碍物的距离"""
        if self.inflated_map is None or self.occupancy_grid is None:
            return float('inf')
        
        resolution = self.occupancy_grid.info.resolution
        origin_x = self.occupancy_grid.info.origin.position.x
        origin_y = self.occupancy_grid.info.origin.position.y
        width = self.occupancy_grid.info.width
        height = self.occupancy_grid.info.height
        
        # 转换为栅格坐标
        center_grid_x = int((x - origin_x) / resolution)
        center_grid_y = int((y - origin_y) / resolution)
        
        # 检查边界
        if not (0 <= center_grid_x < width and 0 <= center_grid_y < height):
            return 0.0
        
        min_distance = float('inf')
        search_radius = int(3.0 / resolution)  # 搜索半径3米
        
        # 在搜索范围内寻找最近的障碍物
        for dy in range(-search_radius, search_radius + 1):
            for dx in range(-search_radius, search_radius + 1):
                check_x = center_grid_x + dx
                check_y = center_grid_y + dy
                
                # 检查边界
                if not (0 <= check_x < width and 0 <= check_y < height):
                    continue
                
                # 如果是障碍物
                if self.inflated_map[check_y, check_x] >= self.obstacle_threshold:
                    distance = math.sqrt(dx*dx + dy*dy) * resolution
                    min_distance = min(min_distance, distance)
        
        return min_distance
    
    def is_path_clear_with_margin(self, start_x: float, start_y: float, end_x: float, end_y: float, safety_margin: float) -> bool:
        """检查路径是否无障碍物（带安全边距）"""
        if self.inflated_map is None:
            return True
        
        resolution = self.occupancy_grid.info.resolution
        origin_x = self.occupancy_grid.info.origin.position.x
        origin_y = self.occupancy_grid.info.origin.position.y
        width = self.occupancy_grid.info.width
        height = self.occupancy_grid.info.height
        
        # 转换为栅格坐标
        start_grid_x = int((start_x - origin_x) / resolution)
        start_grid_y = int((start_y - origin_y) / resolution)
        end_grid_x = int((end_x - origin_x) / resolution)
        end_grid_y = int((end_y - origin_y) / resolution)
        
        # 计算安全边距对应的栅格数
        safety_pixels = int(safety_margin / resolution)
        
        # Bresenham直线算法
        dx = abs(end_grid_x - start_grid_x)
        dy = abs(end_grid_y - start_grid_y)
        x = start_grid_x
        y = start_grid_y
        
        x_inc = 1 if end_grid_x > start_grid_x else -1
        y_inc = 1 if end_grid_y > start_grid_y else -1
        error = dx - dy
        
        dx *= 2
        dy *= 2
        
        while True:
            # 检查当前点周围的安全区域
            for check_dy in range(-safety_pixels, safety_pixels + 1):
                for check_dx in range(-safety_pixels, safety_pixels + 1):
                    check_x = x + check_dx
                    check_y = y + check_dy
                    
                    # 检查边界
                    if not (0 <= check_x < width and 0 <= check_y < height):
                        return False
                    
                    # 检查是否在障碍物中
                    if self.inflated_map[check_y, check_x] >= self.obstacle_threshold:
                        return False
            
            if x == end_grid_x and y == end_grid_y:
                break
                
            if error > 0:
                x += x_inc
                error -= dy
            else:
                y += y_inc
                error += dx
        
        return True
    
    def is_point_safe(self, x: float, y: float) -> bool:
        """检查点是否安全（不在障碍物中）"""
        if self.inflated_map is None or self.occupancy_grid is None:
            return True
        
        resolution = self.occupancy_grid.info.resolution
        origin_x = self.occupancy_grid.info.origin.position.x
        origin_y = self.occupancy_grid.info.origin.position.y
        width = self.occupancy_grid.info.width
        height = self.occupancy_grid.info.height
        
        # 转换为栅格坐标
        grid_x = int((x - origin_x) / resolution)
        grid_y = int((y - origin_y) / resolution)
        
        # 检查边界
        if not (0 <= grid_x < width and 0 <= grid_y < height):
            return False
        
        # 检查是否在障碍物中
        return self.inflated_map[grid_y, grid_x] < self.obstacle_threshold
    
    def find_safe_temp_goal(self, target_x: float, target_y: float) -> Optional[Tuple[float, float]]:
        """寻找安全的临时目标点 - 新的智能算法"""
        if self.current_pose is None:
            return None
        
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        
        # 计算到目标的距离和方向
        distance_to_target = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
        target_angle = math.atan2(target_y - current_y, target_x - current_x)
        
        # 动态计算临时目标距离
        dynamic_temp_distance = self.calculate_dynamic_temp_goal_distance(target_x, target_y)
        
        # 如果目标很近，直接尝试到达
        if distance_to_target <= dynamic_temp_distance:
            if self.check_passage_to_point(current_x, current_y, target_x, target_y):
                return (target_x, target_y)
        
        # 尝试在目标方向上找临时目标点
        temp_goal = self.find_temp_goal_in_direction(current_x, current_y, target_angle, dynamic_temp_distance)
        if temp_goal:
            return temp_goal
        
        # 如果直接方向不行，尝试多个角度偏移
        return self.find_temp_goal_with_angle_search(current_x, current_y, target_angle, dynamic_temp_distance)
    
    def calculate_dynamic_temp_goal_distance(self, target_x: float, target_y: float) -> float:
        """动态计算临时目标距离 - 空旷地带距离更远"""
        if self.current_pose is None:
            return self.temp_goal_distance
        
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        
        # 检查当前位置周围的空旷程度
        openness_factor = self.calculate_area_openness(current_x, current_y, 2.0)
        
        # 根据空旷程度动态调整距离
        if openness_factor > 0.8:  # 非常空旷
            dynamic_distance = self.max_temp_goal_distance
        elif openness_factor > 0.6:  # 比较空旷
            dynamic_distance = self.temp_goal_distance * 1.5
        elif openness_factor > 0.4:  # 一般空旷
            dynamic_distance = self.temp_goal_distance
        else:  # 比较拥挤
            dynamic_distance = self.min_temp_goal_distance
        
        # 如果之前的临时目标失败，增加距离
        if self.temp_goal_retry_count > 0:
            dynamic_distance = min(dynamic_distance * (1 + 0.3 * self.temp_goal_retry_count), 
                                 self.max_temp_goal_distance)
        
        return dynamic_distance
    
    def calculate_area_openness(self, center_x: float, center_y: float, radius: float) -> float:
        """计算某个区域的空旷程度（0-1，1表示完全空旷）"""
        if self.inflated_map is None or self.occupancy_grid is None:
            return 0.5  # 默认中等空旷
        
        resolution = self.occupancy_grid.info.resolution
        origin_x = self.occupancy_grid.info.origin.position.x
        origin_y = self.occupancy_grid.info.origin.position.y
        width = self.occupancy_grid.info.width
        height = self.occupancy_grid.info.height
        
        # 转换为栅格坐标
        center_grid_x = int((center_x - origin_x) / resolution)
        center_grid_y = int((center_y - origin_y) / resolution)
        
        search_radius_pixels = int(radius / resolution)
        
        total_cells = 0
        free_cells = 0
        
        # 检查圆形区域内的栅格
        for dy in range(-search_radius_pixels, search_radius_pixels + 1):
            for dx in range(-search_radius_pixels, search_radius_pixels + 1):
                # 只检查圆形区域内的点
                if dx*dx + dy*dy <= search_radius_pixels*search_radius_pixels:
                    check_x = center_grid_x + dx
                    check_y = center_grid_y + dy
                    
                    # 检查边界
                    if 0 <= check_x < width and 0 <= check_y < height:
                        total_cells += 1
                        if self.inflated_map[check_y, check_x] < self.obstacle_threshold:
                            free_cells += 1
        
        if total_cells == 0:
            return 0.0
        
        return free_cells / total_cells
    
    def find_temp_goal_in_direction(self, start_x: float, start_y: float, 
                                   direction: float, distance: float) -> Optional[Tuple[float, float]]:
        """在指定方向上寻找临时目标点"""
        # 计算目标点
        temp_x = start_x + distance * math.cos(direction)
        temp_y = start_y + distance * math.sin(direction)
        
        # 检查是否有足够宽的通道
        if self.check_passage_to_point(start_x, start_y, temp_x, temp_y):
            return (temp_x, temp_y)
        
        return None
    
    def find_temp_goal_with_angle_search(self, start_x: float, start_y: float, 
                                        target_direction: float, distance: float) -> Optional[Tuple[float, float]]:
        """通过角度搜索寻找临时目标点"""
        # 搜索角度范围从小到大
        angle_increments = [
            math.pi/12,   # ±15度
            math.pi/8,    # ±22.5度
            math.pi/6,    # ±30度
            math.pi/4,    # ±45度
            math.pi/3,    # ±60度
            math.pi/2,    # ±90度
        ]
        
        # 尝试不同的距离（从远到近）
        distances = [
            distance,
            distance * 0.8,
            distance * 0.6,
            distance * 0.4,
        ]
        
        for dist in distances:
            for angle_inc in angle_increments:
                # 先尝试左右两侧
                for side_multiplier in [1, -1]:
                    test_angle = target_direction + side_multiplier * angle_inc
                    temp_x = start_x + dist * math.cos(test_angle)
                    temp_y = start_y + dist * math.sin(test_angle)
                    
                    if self.check_passage_to_point(start_x, start_y, temp_x, temp_y):
                        return (temp_x, temp_y)
        
        return None
    
    def check_passage_to_point(self, start_x: float, start_y: float, 
                              end_x: float, end_y: float) -> bool:
        """检查从起点到终点是否有足够宽的通道（基于0.6m通道宽度）"""
        if self.inflated_map is None or self.occupancy_grid is None:
            return True
        
        resolution = self.occupancy_grid.info.resolution
        origin_x = self.occupancy_grid.info.origin.position.x
        origin_y = self.occupancy_grid.info.origin.position.y
        width = self.occupancy_grid.info.width
        height = self.occupancy_grid.info.height
        
        # 计算路径方向和垂直方向
        path_length = math.sqrt((end_x - start_x)**2 + (end_y - start_y)**2)
        if path_length < 0.01:
            return True
        
        path_dir_x = (end_x - start_x) / path_length
        path_dir_y = (end_y - start_y) / path_length
        
        # 垂直于路径的方向（用于检查通道宽度）
        perp_dir_x = -path_dir_y
        perp_dir_y = path_dir_x
        
        # 通道半宽度
        half_width = self.min_passage_width / 2.0
        
        # 沿路径检查多个点
        num_checks = max(5, int(path_length / (resolution * 2)))
        
        for i in range(num_checks + 1):
            t = i / num_checks
            check_x = start_x + t * (end_x - start_x)
            check_y = start_y + t * (end_y - start_y)
            
            # 检查通道两侧是否有足够空间
            if not self.check_passage_width_at_point(check_x, check_y, perp_dir_x, perp_dir_y, half_width):
                return False
        
        return True
    
    def check_passage_width_at_point(self, center_x: float, center_y: float,
                                    perp_dir_x: float, perp_dir_y: float, half_width: float) -> bool:
        """检查某点处的通道宽度是否足够"""
        if self.inflated_map is None or self.occupancy_grid is None:
            return True
        
        resolution = self.occupancy_grid.info.resolution
        origin_x = self.occupancy_grid.info.origin.position.x
        origin_y = self.occupancy_grid.info.origin.position.y
        width = self.occupancy_grid.info.width
        height = self.occupancy_grid.info.height
        
        # 检查中心点本身
        center_grid_x = int((center_x - origin_x) / resolution)
        center_grid_y = int((center_y - origin_y) / resolution)
        
        if not (0 <= center_grid_x < width and 0 <= center_grid_y < height):
            return False
        
        if self.inflated_map[center_grid_y, center_grid_x] >= self.obstacle_threshold:
            return False
        
        # 检查通道两侧
        num_side_checks = max(3, int(half_width / resolution))
        
        for side in [-1, 1]:  # 左右两侧
            for i in range(1, num_side_checks + 1):
                check_distance = i * resolution
                if check_distance > half_width:
                    break
                
                check_x = center_x + side * check_distance * perp_dir_x
                check_y = center_y + side * check_distance * perp_dir_y
                
                check_grid_x = int((check_x - origin_x) / resolution)
                check_grid_y = int((check_y - origin_y) / resolution)
                
                if not (0 <= check_grid_x < width and 0 <= check_grid_y < height):
                    return False
                
                if self.inflated_map[check_grid_y, check_grid_x] >= self.obstacle_threshold:
                    return False
        
        return True
    
    def check_robot_movement(self) -> bool:
        """检查机器人是否在移动"""
        if self.current_pose is None:
            return False
        
        current_time = time.time()
        current_position = (self.current_pose.position.x, self.current_pose.position.y)
        
        # 如果没有上次位置记录，记录当前位置
        if self.last_position is None:
            self.last_position = current_position
            self.last_position_check_time = current_time
            return True  # 假设正在移动
        
        # 检查时间间隔
        time_elapsed = current_time - self.last_position_check_time
        if time_elapsed < 2.0:  # 至少等待2秒再检查
            return True
        
        # 计算移动距离
        distance_moved = math.sqrt(
            (current_position[0] - self.last_position[0])**2 + 
            (current_position[1] - self.last_position[1])**2
        )
        
        # 更新位置记录
        self.last_position = current_position
        self.last_position_check_time = current_time
        
        # 判断是否移动
        is_moving = distance_moved > self.robot_movement_threshold
        
        if not is_moving:
            self.get_logger().warn(f'Robot not moving! Distance in {time_elapsed:.1f}s: {distance_moved:.3f}m')
        
        return is_moving
    
    def reset_temp_goal_retry(self):
        """重置临时目标重试计数"""
        self.temp_goal_retry_count = 0
    
    def increment_temp_goal_retry(self):
        """增加临时目标重试计数"""
        self.temp_goal_retry_count += 1
        self.get_logger().info(f'Temp goal retry count: {self.temp_goal_retry_count}')
    
    def is_progress_towards_goal(self, temp_x: float, temp_y: float, target_x: float, target_y: float) -> bool:
        """检查临时目标点是否能让我们朝着最终目标前进"""
        if self.current_pose is None:
            return True
        
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        
        # 计算当前到目标的距离
        current_to_target = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
        
        # 计算临时目标到最终目标的距离
        temp_to_target = math.sqrt((target_x - temp_x)**2 + (target_y - temp_y)**2)
        
        # 如果临时目标点让我们更接近最终目标，或者相差不大，则认为是进步
        return temp_to_target <= current_to_target + 0.5
    
    def publish_temp_goal(self, temp_x: float, temp_y: float):
        """发布临时目标点"""
        temp_goal = Point()
        temp_goal.x = temp_x
        temp_goal.y = temp_y
        temp_goal.z = 0.0
        
        self.temp_goal_publisher.publish(temp_goal)
        self.current_temp_goal = (temp_x, temp_y)
        self.last_temp_goal_time = time.time()
        
        self.get_logger().info(f'Published temp goal: ({temp_x:.2f}, {temp_y:.2f})')
    
    def recovery_behavior(self):
        """恢复行为：重新规划路径"""
        self.get_logger().warn('Robot appears to be stuck, executing recovery behavior')
        
        # 清空位置历史
        self.position_history.clear()
        
        # 重置临时目标
        self.current_temp_goal = None
        
        # 重新规划路径
        self.plan_path()
        self.control_state = "NAVIGATING"
        self.last_replan_time = time.time()
    
    def control_loop(self):
        """主控制循环 - 基于新的智能临时目标点导航"""
        if not self.is_navigating or self.current_pose is None:
            return
        
        current_time = time.time()
        
        # 检查是否到达最终目标
        distance_to_final_goal = self.distance_to_goal()
        if distance_to_final_goal < self.goal_tolerance:
            self.is_navigating = False
            self.control_state = "REACHED"
            self.reset_temp_goal_retry()
            self.get_logger().info('Goal reached!')
            return
        
        # 检查机器人是否在移动
        robot_is_moving = self.check_robot_movement()
        
        # 如果机器人没有移动且有临时目标，增加重试计数
        if not robot_is_moving and self.current_temp_goal is not None:
            self.increment_temp_goal_retry()
            if self.temp_goal_retry_count >= self.max_temp_goal_retries:
                self.get_logger().warn(f'Max temp goal retries reached ({self.max_temp_goal_retries}), replanning path')
                self.plan_path()
                self.reset_temp_goal_retry()
                self.last_replan_time = current_time
                return
        
        # 卡住检测
        if self.detect_stuck() and current_time - self.last_replan_time > self.replan_cooldown:
            self.control_state = "STUCK"
            self.recovery_behavior()
            return
        
        # 更新当前路径索引（基于机器人位置）
        self.update_path_index()
        
        # 确定下一个临时目标
        next_temp_goal = None
        
        if self.path:
            self.get_logger().info(f'Path has {len(self.path)} waypoints, current index: {self.current_path_index}')
            # 使用路径上的前瞻点
            lookahead_point = self.get_lookahead_point_on_path()
            if lookahead_point:
                self.get_logger().info(f'Found lookahead point: ({lookahead_point[0]:.2f}, {lookahead_point[1]:.2f})')
                next_temp_goal = self.find_safe_temp_goal(lookahead_point[0], lookahead_point[1])
            else:
                self.get_logger().warn('No lookahead point found on path')
        else:
            self.get_logger().warn('No path available for navigation')
        
        if next_temp_goal is None:
            self.get_logger().info('Path method failed, trying direct goal approach')
            # 如果路径方法失败，直接朝向最终目标
            next_temp_goal = self.find_safe_temp_goal(
                self.goal_pose.position.x, 
                self.goal_pose.position.y
            )
        
        # 发布临时目标
        if next_temp_goal:
            temp_x, temp_y = next_temp_goal
            
            # 检查是否需要更新临时目标
            should_update = True
            if self.current_temp_goal:
                # 计算与当前临时目标的距离
                current_temp_x, current_temp_y = self.current_temp_goal
                distance_to_current_temp = math.sqrt(
                    (temp_x - current_temp_x)**2 + (temp_y - current_temp_y)**2
                )
                
                # 计算机器人到当前临时目标的距离
                robot_to_current_temp = math.sqrt(
                    (current_temp_x - self.current_pose.position.x)**2 + 
                    (current_temp_y - self.current_pose.position.y)**2
                )
                
                # 如果新目标与当前目标很接近，机器人正在移动，且机器人还没到达当前目标，则不更新
                if (distance_to_current_temp < 1.0 and 
                    robot_to_current_temp > 0.8 and  # 机器人还没到达当前目标
                    robot_is_moving):  # 机器人正在移动
                    should_update = False
            
            if should_update:
                self.publish_temp_goal(temp_x, temp_y)
                self.reset_temp_goal_retry()  # 重置重试计数
                openness = self.calculate_area_openness(self.current_pose.position.x, self.current_pose.position.y, 2.0)
                self.get_logger().info(f'Published temp goal: ({temp_x:.2f}, {temp_y:.2f}), area openness: {openness:.2f}, retry count reset')
        else:
            # 没有找到安全的临时目标，重新规划
            if current_time - self.last_replan_time > self.replan_cooldown:
                distance_to_goal = math.sqrt(
                    (self.goal_pose.position.x - self.current_pose.position.x)**2 + 
                    (self.goal_pose.position.y - self.current_pose.position.y)**2
                )
                self.get_logger().warn(f'No safe temp goal found (distance to goal: {distance_to_goal:.2f}m), replanning path')
                self.plan_path()
                self.reset_temp_goal_retry()
                self.last_replan_time = current_time
    
    def update_path_index(self):
        """更新当前路径索引"""
        if not self.path or self.current_pose is None:
            return
        
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        
        # 查找离机器人最近的路径点
        min_distance = float('inf')
        closest_index = self.current_path_index
        
        # 从当前索引开始向前搜索
        for i in range(self.current_path_index, len(self.path)):
            path_x, path_y = self.path[i]
            distance = math.sqrt((path_x - current_x)**2 + (path_y - current_y)**2)
            
            if distance < min_distance:
                min_distance = distance
                closest_index = i
            
            # 如果距离开始增大，说明已经找到最近点
            if distance > min_distance + 0.5:
                break
        
        # 更新路径索引
        if closest_index > self.current_path_index:
            self.current_path_index = closest_index
            self.get_logger().debug(f'Updated path index to {self.current_path_index}')


def main(args=None):
    rclpy.init(args=args)
    
    node = SimpleNavigationController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
