#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid, Odometry, Path
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
        self.declare_parameter('path_smoothing_factor', 0.5) # 路径平滑因子
        self.declare_parameter('waypoint_spacing', 0.5) # 路径点间距
    # 添加临时目标相关参数（精简保留）
        self.declare_parameter('temp_goal_update_frequency', 2.0) # 临时目标更新频率
        self.declare_parameter('min_temp_goal_publish_distance', 0.5) # 发布临时目标的最小距离（避免目标太近被认为已到达）
        
        # 获取参数
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.control_frequency = self.get_parameter('control_frequency').value
        self.obstacle_inflation_radius = self.get_parameter('obstacle_inflation_radius').value
        self.max_planning_time = self.get_parameter('max_planning_time').value
        self.path_smoothing_factor = self.get_parameter('path_smoothing_factor').value
        self.waypoint_spacing = self.get_parameter('waypoint_spacing').value
        self.temp_goal_update_frequency = self.get_parameter('temp_goal_update_frequency').value
        self.min_temp_goal_publish_distance = self.get_parameter('min_temp_goal_publish_distance').value
        # 新的队列化临时目标参数
        self.declare_parameter('temp_goal_spacing', 0.5)  # 临时目标之间最小间距（m）
        self.declare_parameter('temp_path_safety_distance', 0.3)  # 路径到障碍物最小距离（m）
        self.declare_parameter('goal_reached_distance', 0.3)  # 到达最终目标阈值（m）
        self.temp_goal_spacing = self.get_parameter('temp_goal_spacing').value
        self.temp_path_safety_distance = self.get_parameter('temp_path_safety_distance').value
        self.goal_reached_distance = self.get_parameter('goal_reached_distance').value

        # 队列生成鲁棒性参数
        self.declare_parameter('allow_partial_temp_queue', True)  # 若部分点无效，是否保留已验证的前缀，不整段失败
        self.declare_parameter('temp_queue_adjust_backoff_ratio', 0.2)  # 回退比例（相对 spacing）
        self.declare_parameter('temp_queue_adjust_max_steps', 5)  # 最大回退步数
        self.declare_parameter('temp_queue_min_spacing_factor', 0.8)  # 允许的最小间距因子
        self.declare_parameter('temp_queue_lateral_offset_max', 0.5)  # 横向最大偏移（m）
        self.declare_parameter('temp_queue_lateral_offset_step', 0.1)  # 横向偏移步长（m）
        self.allow_partial_temp_queue = self.get_parameter('allow_partial_temp_queue').value
        self.temp_queue_adjust_backoff_ratio = float(self.get_parameter('temp_queue_adjust_backoff_ratio').value)
        self.temp_queue_adjust_max_steps = int(self.get_parameter('temp_queue_adjust_max_steps').value)
        self.temp_queue_min_spacing_factor = float(self.get_parameter('temp_queue_min_spacing_factor').value)
        self.temp_queue_lateral_offset_max = float(self.get_parameter('temp_queue_lateral_offset_max').value)
        self.temp_queue_lateral_offset_step = float(self.get_parameter('temp_queue_lateral_offset_step').value)

        # 全局临时目标队列和地图更新标志
        self.temp_goal_queue = deque()
        self.map_updated = False
        self.last_goal_coords = None
        
        # 状态变量
        self.current_pose = None
        self.goal_pose = None
        self.occupancy_grid = None
        self.is_navigating = False
        
        self.current_temp_goal = None  # 当前临时目标
        
        
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
        
        # 创建临时目标队列路径发布者
        self.temp_goal_path_publisher = self.create_publisher(
            Path,
            '/temp_goal_path',
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

        # 创建定时器：使用更高的控制频率以提升响应与切换流畅度
        eff_freq = max(float(self.control_frequency), float(self.temp_goal_update_frequency))
        timer_period = 1.0 / eff_freq
        self.control_timer = self.create_timer(timer_period, self.control_loop)
        
        self.get_logger().info('Simple Navigation Controller started - Enhanced Version')
        self.get_logger().info(f'Goal tolerance: {self.goal_tolerance} m')
        self.get_logger().info(f'Temp goal update frequency: {self.temp_goal_update_frequency} Hz')
        self.get_logger().info(f'Min temp goal publish distance: {self.min_temp_goal_publish_distance} m')
        self.get_logger().info(f'Temp goal spacing: {self.temp_goal_spacing} m')
        self.get_logger().info(f'Temp path safety distance: {self.temp_path_safety_distance} m')
        self.get_logger().info(f'Goal reached distance: {self.goal_reached_distance} m')
        self.get_logger().info(f'Allow partial temp queue: {self.allow_partial_temp_queue}')
        self.get_logger().info(f'Queue adjust backoff ratio: {self.temp_queue_adjust_backoff_ratio}')
        self.get_logger().info(f'Queue adjust max steps: {self.temp_queue_adjust_max_steps}')
        self.get_logger().info(f'Queue min spacing factor: {self.temp_queue_min_spacing_factor}')
        self.get_logger().info(f'Queue lateral offset max: {self.temp_queue_lateral_offset_max} m')
        self.get_logger().info(f'Queue lateral offset step: {self.temp_queue_lateral_offset_step} m')
        
    def goal_callback(self, msg: PoseStamped):
        """目标点回调函数"""
        self.goal_pose = msg.pose
        self.is_navigating = True
        self.current_temp_goal = None  # 重置临时目标

        goal_x = self.goal_pose.position.x
        goal_y = self.goal_pose.position.y

        self.get_logger().info(f'New goal received: ({goal_x:.2f}, {goal_y:.2f})')
        # 不在回调中单独规划/发布路径，交由控制循环基于队列处理
        
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
        # 标记地图更新，可能需要重新生成临时目标队列
        self.map_updated = True
    
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

    def publish_temp_goal_queue_path(self):
        """将当前临时目标队列发布为 Path，便于 RViz 可视化"""
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for x, y in list(self.temp_goal_queue):
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.header.stamp = path_msg.header.stamp
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.w = 1.0
            path_msg.poses.append(pose_stamped)

        self.temp_goal_path_publisher.publish(path_msg)
    
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
        """简化版安全检查：
        - 点安全：临时点与障碍物的最小安全距离 >= temp_path_safety_distance
        - 线安全：机器人当前位置到临时点的直线段具备相同安全边距
        """
        if self.current_pose is None:
            return False

        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y

        margin = float(self.temp_path_safety_distance)
        return (
            self.is_point_safe_with_margin(temp_x, temp_y, margin) and
            self.is_path_clear_with_margin(current_x, current_y, temp_x, temp_y, margin)
        )
    
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
        # 旧逻辑保留但不再作为主流程使用
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

    def sample_path_into_waypoints(self, path_points: List[Tuple[float, float]], spacing: float) -> List[Tuple[float, float]]:
        """把一段路径（路径点列表）按 spacing 进行采样，返回采样后的 waypoint 列表（包含起点和终点）。
        间距尽量接近 spacing，且不会低于 spacing（除终点外）。"""
        if not path_points:
            return []

        waypoints: List[Tuple[float, float]] = []
        # 使用路径的第一个点作为起点
        waypoints.append(path_points[0])

        for i in range(1, len(path_points)):
            p0 = path_points[i-1]
            p1 = path_points[i]
            dx = p1[0] - p0[0]
            dy = p1[1] - p0[1]
            seg_len = math.hypot(dx, dy)
            if seg_len <= 1e-6:
                continue

            # 计算该段需要多少等分（尽量保证每段长度不小于 spacing）
            num = max(1, int(math.floor(seg_len / spacing)))
            for k in range(1, num + 1):
                t = k / num
                x = p0[0] + t * dx
                y = p0[1] + t * dy
                # 只有当与上一个采样点距离不小于 spacing - 1e-6 才添加
                if math.hypot(x - waypoints[-1][0], y - waypoints[-1][1]) + 1e-6 >= spacing:
                    waypoints.append((x, y))

        # 确保终点存在
        if math.hypot(waypoints[-1][0] - path_points[-1][0], waypoints[-1][1] - path_points[-1][1]) > 1e-6:
            waypoints.append(path_points[-1])

        return waypoints

    def generate_temp_goal_queue(self, goal_x: float, goal_y: float) -> List[Tuple[float, float]]:
        """根据当前位姿和地图生成完整的临时目标点队列，满足间距和安全距离要求。
        返回生成的点列表（不含机器人当前位置）。"""
        self.get_logger().info('Generating full temp goal queue...')

        if self.current_pose is None:
            self.get_logger().warn('Cannot generate temp goals: current_pose unknown')
            return []

        if self.occupancy_grid is None or self.inflated_map is None:
            self.get_logger().warn('Cannot generate temp goals: map not available')
            return []

        start_x = self.current_pose.position.x
        start_y = self.current_pose.position.y

        # 如果直接连线满足安全与可通行性，则直接从直线生成
        if self.is_path_clear_with_margin(start_x, start_y, goal_x, goal_y, self.temp_path_safety_distance):
            # 直接从起点到终点按 spacing 采样
            total_dx = goal_x - start_x
            total_dy = goal_y - start_y
            total_dist = math.hypot(total_dx, total_dy)
            if total_dist < 1e-6:
                return []

            n_segments = max(1, int(math.floor(total_dist / self.temp_goal_spacing)))
            points = []
            for k in range(1, n_segments + 1):
                t = k / n_segments
                x = start_x + t * total_dx
                y = start_y + t * total_dy
                points.append((x, y))

            # 确保最后一个点为最终目标
            if math.hypot(points[-1][0] - goal_x, points[-1][1] - goal_y) > 1e-6:
                points.append((goal_x, goal_y))

            # 过滤每个点是否在安全区内
            filtered = []
            prev = (start_x, start_y)
            for p in points:
                if not self.is_point_safe_with_margin(p[0], p[1], self.temp_path_safety_distance):
                    self.get_logger().warn(f'Point {p} too close to obstacle, abort generation')
                    return []
                if not self.is_path_clear_with_margin(prev[0], prev[1], p[0], p[1], self.temp_path_safety_distance):
                    self.get_logger().warn(f'Segment {prev}->{p} not safe, abort generation')
                    return []
                filtered.append(p)
                prev = p

            return filtered

        # 否则尝试使用 A* 规划一条路径，然后对路径采样
        path_points = self.astar_planning(start_x, start_y, goal_x, goal_y)
        if not path_points:
            self.get_logger().warn('A* failed to find path for temp goal queue generation')
            # 尝试退化为单步前进目标
            fallback = self._find_safe_progress_point((start_x, start_y), (goal_x, goal_y))
            if fallback is not None:
                self.get_logger().warn('Using fallback single progress point due to A* failure')
                return [fallback]
            return []

        # 平滑并采样路径
        smoothed = self.smooth_path(path_points)
        sampled = self.sample_path_into_waypoints(smoothed, self.temp_goal_spacing)

        # 验证采样点与相邻连线的安全性（带回退与部分保留）
        validated: List[Tuple[float, float]] = []
        prev = (start_x, start_y)
        min_spacing = max(0.05, self.temp_goal_spacing * self.temp_queue_min_spacing_factor)
        backoff_step = max(0.02, self.temp_goal_spacing * self.temp_queue_adjust_backoff_ratio)

        def try_lateral_offsets(base_prev: Tuple[float, float], base_p: Tuple[float, float]) -> Optional[Tuple[float, float]]:
            """围绕 prev->p 方向，在 p 附近做横向偏移搜索，返回第一个安全候选点。"""
            px, py = base_prev
            tx, ty = base_p
            vx = tx - px
            vy = ty - py
            seg_len = math.hypot(vx, vy)
            if seg_len < 1e-6:
                return None
            ux, uy = vx / seg_len, vy / seg_len
            # 法向
            nx, ny = -uy, ux
            # 以偏移距离从小到大、左右交替搜索
            max_off = self.temp_queue_lateral_offset_max
            step = max(1e-3, self.temp_queue_lateral_offset_step)
            k = 1
            off = step
            while off <= max_off + 1e-9:
                for s in (1, -1):
                    cx = tx + s * nx * off
                    cy = ty + s * ny * off
                    # 保证与 prev 的最小间距
                    if math.hypot(cx - px, cy - py) + 1e-6 < min_spacing:
                        continue
                    if self.is_point_safe_with_margin(cx, cy, self.temp_path_safety_distance) and \
                       self.is_path_clear_with_margin(px, py, cx, cy, self.temp_path_safety_distance):
                        return (cx, cy)
                k += 1
                off = k * step
            return None

        for idx, p in enumerate(sampled):
            # 跳过可能等于起点的点
            if math.hypot(p[0]-prev[0], p[1]-prev[1]) < 1e-6:
                prev = p
                continue

            ok_point = self.is_point_safe_with_margin(p[0], p[1], self.temp_path_safety_distance)
            ok_seg = self.is_path_clear_with_margin(prev[0], prev[1], p[0], p[1], self.temp_path_safety_distance)

            candidate = p
            # 若点或段不安全，尝试沿 prev->p 方向回退若干
            if not (ok_point and ok_seg):
                dir_x = p[0] - prev[0]
                dir_y = p[1] - prev[1]
                seg_len = math.hypot(dir_x, dir_y)
                if seg_len > 1e-6:
                    dir_x /= seg_len
                    dir_y /= seg_len
                adjusted = False
                for step in range(1, self.temp_queue_adjust_max_steps + 1):
                    new_len = seg_len - step * backoff_step
                    # 保证与 prev 的距离不低于 min_spacing（避免过密）
                    if new_len < min_spacing:
                        break
                    cand_x = prev[0] + dir_x * new_len
                    cand_y = prev[1] + dir_y * new_len
                    if self.is_point_safe_with_margin(cand_x, cand_y, self.temp_path_safety_distance) and \
                       self.is_path_clear_with_margin(prev[0], prev[1], cand_x, cand_y, self.temp_path_safety_distance):
                        candidate = (cand_x, cand_y)
                        adjusted = True
                        break
                if not adjusted:
                    # 尝试横向偏移在 p 附近寻找安全候选
                    lateral = try_lateral_offsets(prev, p)
                    if lateral is not None:
                        candidate = lateral
                        adjusted = True
                if not adjusted:
                    # 如果无法调整该点
                    if self.allow_partial_temp_queue and validated:
                        self.get_logger().warn(f'Sampled point {p} invalid; keep partial queue with {len(validated)} waypoints')
                        break
                    else:
                        self.get_logger().warn(f'Sampled point {p} invalid and no partial allowed; abort generation')
                        return []

            # 通过验证，加入队列
            validated.append(candidate)
            prev = candidate

        # 确保最后一个点是最终目标
        if validated and math.hypot(validated[-1][0]-goal_x, validated[-1][1]-goal_y) > 1e-3:
            # 对最终段也做一次安全验证/回退
            p = (goal_x, goal_y)
            ok_point = self.is_point_safe_with_margin(p[0], p[1], self.temp_path_safety_distance)
            ok_seg = self.is_path_clear_with_margin(prev[0], prev[1], p[0], p[1], self.temp_path_safety_distance)
            candidate = p
            if not (ok_point and ok_seg):
                dir_x = p[0] - prev[0]
                dir_y = p[1] - prev[1]
                seg_len = math.hypot(dir_x, dir_y)
                if seg_len > 1e-6:
                    dir_x /= seg_len
                    dir_y /= seg_len
                adjusted = False
                for step in range(1, self.temp_queue_adjust_max_steps + 1):
                    new_len = seg_len - step * backoff_step
                    if new_len < min_spacing:
                        break
                    cand_x = prev[0] + dir_x * new_len
                    cand_y = prev[1] + dir_y * new_len
                    if self.is_point_safe_with_margin(cand_x, cand_y, self.temp_path_safety_distance) and \
                       self.is_path_clear_with_margin(prev[0], prev[1], cand_x, cand_y, self.temp_path_safety_distance):
                        candidate = (cand_x, cand_y)
                        adjusted = True
                        break
                if not adjusted:
                    # 尝试横向偏移
                    lateral = try_lateral_offsets(prev, p)
                    if lateral is not None:
                        candidate = lateral
                        adjusted = True
                if not adjusted:
                    if self.allow_partial_temp_queue and validated:
                        self.get_logger().warn('Final goal segment invalid; keep partial queue')
                    else:
                        self.get_logger().warn('Final goal segment invalid and no partial allowed; returning empty')
                        return []
            validated.append(candidate)

        # 若没有任何有效点，尝试单步前进点作为兜底
        if not validated:
            fallback = self._find_safe_progress_point((start_x, start_y), (goal_x, goal_y))
            if fallback is not None:
                self.get_logger().warn('Validated queue empty; using fallback single progress point')
                return [fallback]
            return []

        return validated

    def _find_safe_progress_point(self, start: Tuple[float, float], goal: Tuple[float, float]) -> Optional[Tuple[float, float]]:
        """从 start 朝 goal 方向寻找一个安全的前进点（>=最小发布距离），支持横向偏移兜底。
        优先距离：使用 max(min_temp_goal_publish_distance, temp_goal_spacing)。
        若直线不安全，按横向偏移逐步搜索。
        返回找到的 (x,y) 或 None。
        """
        if self.inflated_map is None or self.occupancy_grid is None:
            return None

        sx, sy = start
        gx, gy = goal
        vx = gx - sx
        vy = gy - sy
        dist = math.hypot(vx, vy)
        if dist < 1e-6:
            return None
        ux, uy = vx / dist, vy / dist

        d = max(float(self.min_temp_goal_publish_distance), float(self.temp_goal_spacing))
        d = min(d, max(0.5, dist))
        cx = sx + ux * d
        cy = sy + uy * d

        # 直线候选
        if self.is_point_safe_with_margin(cx, cy, self.temp_path_safety_distance) and \
           self.is_path_clear_with_margin(sx, sy, cx, cy, self.temp_path_safety_distance):
            return (cx, cy)

        # 横向偏移搜索
        nx, ny = -uy, ux
        max_off = self.temp_queue_lateral_offset_max
        step = max(1e-3, self.temp_queue_lateral_offset_step)
        k = 1
        off = step
        while off <= max_off + 1e-9:
            for s in (1, -1):
                tx = cx + s * nx * off
                ty = cy + s * ny * off
                if self.is_point_safe_with_margin(tx, ty, self.temp_path_safety_distance) and \
                   self.is_path_clear_with_margin(sx, sy, tx, ty, self.temp_path_safety_distance):
                    return (tx, ty)
            k += 1
            off = k * step
        return None
    
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

    def _ensure_head_min_distance(self, min_dist: float) -> bool:
        """确保队列头部目标与机器人距离不小于 min_dist。
        策略：
        - 连续跳过所有小于 min_dist 的头部点（若后续存在且可行）；
        - 若无法跳过，则尝试用从机器人位置沿头部方向的 min_dist 点替换队列头（需安全且连线安全）；
        - 若最终队列为空则返回 False。
        修改 self.temp_goal_queue 并在变更后发布可视化 Path。
        """
        if not self.temp_goal_queue or self.current_pose is None:
            return False

        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y

        changed = False
        while self.temp_goal_queue:
            hx, hy = self.temp_goal_queue[0]
            dist = math.hypot(hx - robot_x, hy - robot_y)
            if dist + 1e-6 >= min_dist:
                break

            # 有第二个点且更远，优先跳过当前头
            if len(self.temp_goal_queue) >= 2:
                sx, sy = self.temp_goal_queue[1]
                sdist = math.hypot(sx - robot_x, sy - robot_y)
                # 检查到第二个点的路径安全
                path_ok = self.is_path_clear_with_margin(robot_x, robot_y, sx, sy, self.temp_path_safety_distance)
                if sdist + 1e-6 >= min_dist and path_ok:
                    self.temp_goal_queue.popleft()
                    changed = True
                    continue

            # 尝试将头部替换为沿机器人->头部方向的 min_dist 点
            dx = hx - robot_x
            dy = hy - robot_y
            seg = math.hypot(dx, dy)
            if seg < 1e-6:
                # 点与机器人重合，直接丢弃该点
                self.temp_goal_queue.popleft()
                changed = True
                continue
            ux, uy = dx / seg, dy / seg
            nx = robot_x + ux * min_dist
            ny = robot_y + uy * min_dist
            # 安全检查
            point_ok = self.is_point_safe_with_margin(nx, ny, self.temp_path_safety_distance)
            path_ok = self.is_path_clear_with_margin(robot_x, robot_y, nx, ny, self.temp_path_safety_distance)
            if point_ok and path_ok:
                # 替换队列头
                self.temp_goal_queue[0] = (nx, ny)
                changed = True
                break
            else:
                # 无法替换，只能移除当前头部继续尝试
                self.temp_goal_queue.popleft()
                changed = True

        if changed:
            self.publish_temp_goal_queue_path()

        return bool(self.temp_goal_queue)
    
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
        """主控制循环 - 使用全局临时目标队列导航"""
        if not self.is_navigating or self.current_pose is None or self.goal_pose is None:
            return

        current_time = time.time()

        # 检查是否到达最终目标（使用更严格的到达阈值）
        distance_to_final_goal = self.distance_to_goal()
        if distance_to_final_goal <= self.goal_reached_distance:
            self.is_navigating = False
            self.control_state = "REACHED"
            self.temp_goal_queue.clear()
            self.current_temp_goal = None
            self.get_logger().info('Final goal reached!')
            return

        goal_x = self.goal_pose.position.x
        goal_y = self.goal_pose.position.y

        # 如果最终目标发生变化，重新生成队列
        if self.last_goal_coords != (goal_x, goal_y):
            self.get_logger().info('Goal changed: regenerating temp goal queue')
            self.temp_goal_queue.clear()
            new_points = self.generate_temp_goal_queue(goal_x, goal_y)
            for p in new_points:
                self.temp_goal_queue.append(p)
            # 发布队列 Path 以便可视化
            self.publish_temp_goal_queue_path()
            self.last_goal_coords = (goal_x, goal_y)
            self.map_updated = False
            # 立即发布第一个临时目标（如果存在且满足最小距离要求0.5m）
            if self.temp_goal_queue:
                if self._ensure_head_min_distance(max(0.5, self.min_temp_goal_publish_distance)):
                    nx, ny = self.temp_goal_queue[0]
                    self.publish_temp_goal(nx, ny)
            return

        # 如果地图更新，验证现有队列是否仍然安全；若不安全则重新生成
        if self.map_updated and self.temp_goal_queue:
            self.get_logger().info('Map updated: validating existing temp goal queue')
            prev = (self.current_pose.position.x, self.current_pose.position.y)
            valid = True
            for p in list(self.temp_goal_queue):
                if not self.is_point_safe_with_margin(p[0], p[1], self.temp_path_safety_distance) or \
                   not self.is_path_clear_with_margin(prev[0], prev[1], p[0], p[1], self.temp_path_safety_distance):
                    valid = False
                    break
                prev = p

            if not valid:
                self.get_logger().info('Existing temp goal queue invalidated by map changes; regenerating')
                self.temp_goal_queue.clear()
                new_points = self.generate_temp_goal_queue(goal_x, goal_y)
                for p in new_points:
                    self.temp_goal_queue.append(p)
                # 发布新的队列 Path
                self.publish_temp_goal_queue_path()
                self.map_updated = False
                if self.temp_goal_queue:
                    nx, ny = self.temp_goal_queue[0]
                    self.publish_temp_goal(nx, ny)
                return
            else:
                # 队列仍然有效，清除地图更新标志
                self.map_updated = False

        # 如果当前没有临时目标，尝试从队列取出并发布
        if self.current_temp_goal is None and self.temp_goal_queue:
            if self._ensure_head_min_distance(max(0.5, self.min_temp_goal_publish_distance)):
                nx, ny = self.temp_goal_queue[0]
                self.publish_temp_goal(nx, ny)

        # 检查机器人到当前临时目标的距离，满足切换条件则切换到下一个
        if self.current_temp_goal is not None:
            robot_x = self.current_pose.position.x
            robot_y = self.current_pose.position.y
            curr_x, curr_y = self.current_temp_goal
            dist = math.hypot(curr_x - robot_x, curr_y - robot_y)
            # 切换阈值：固定为 0.5m，更快切换
            if dist <= 0.5:
                # 弹出当前目标（如果队列头与当前目标一致）
                if self.temp_goal_queue and math.hypot(self.temp_goal_queue[0][0] - curr_x, self.temp_goal_queue[0][1] - curr_y) < 1e-3:
                    self.temp_goal_queue.popleft()
                    # 队列变化，更新可视化 Path
                    self.publish_temp_goal_queue_path()

                # 发布下一个目标或最终目标
                if self.temp_goal_queue:
                    # 确保新目标与机器人至少0.5m
                    if self._ensure_head_min_distance(max(0.5, self.min_temp_goal_publish_distance)):
                        nx, ny = self.temp_goal_queue[0]
                        self.publish_temp_goal(nx, ny)
                else:
                    # 队列为空，优先尝试发布最终目标（若安全）；否则退化为单步前进点
                    if self.check_temp_goal_safety(goal_x, goal_y):
                        self.publish_temp_goal(goal_x, goal_y)
                        # 清空队列 Path 可视化（发布空 Path）
                        self.publish_temp_goal_queue_path()
                    else:
                        fallback = self._find_safe_progress_point((robot_x, robot_y), (goal_x, goal_y))
                        if fallback is not None:
                            fx, fy = fallback
                            self.get_logger().warn('Final goal unsafe; using fallback progress point')
                            self.publish_temp_goal(fx, fy)
                        else:
                            self.get_logger().warn('Final goal not safe to publish; waiting for map updates')
    
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
