#!/usr/bin/env python3
"""
ROS2 Nav2集成导航节点
使用Nav2的全局路径规划器，提取临时目标点供底层控制器使用
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from nav2_msgs.action import ComputePathToPose
import math
from typing import List, Tuple, Optional


class Nav2NavigationNode(Node):
    """
    Nav2导航集成节点
    - 接收目标点
    - 使用Nav2规划全局路径
    - 提取并发布临时目标点
    """
    
    def __init__(self):
        super().__init__('nav2_navigation_node')
        
        # 声明参数
        self.declare_parameter('planner_id', 'GridBased')  # GridBased, Smac2d, NavFn
        self.declare_parameter('temp_goal_spacing', 1.0)  # 临时目标点间距(m)
        self.declare_parameter('temp_goal_switch_distance', 0.6)  # 切换到下一个临时目标的距离(m)
        self.declare_parameter('goal_reached_distance', 0.3)  # 到达最终目标的距离(m)
        self.declare_parameter('replan_frequency', 2.0)  # 重新规划频率(Hz)
        self.declare_parameter('min_path_points', 5)  # 最小路径点数
        
        # 获取参数
        self.planner_id = self.get_parameter('planner_id').value
        self.temp_goal_spacing = self.get_parameter('temp_goal_spacing').value
        self.temp_goal_switch_distance = self.get_parameter('temp_goal_switch_distance').value
        self.goal_reached_distance = self.get_parameter('goal_reached_distance').value
        self.replan_frequency = self.get_parameter('replan_frequency').value
        self.min_path_points = self.get_parameter('min_path_points').value
        
        # 状态变量
        self.current_pose = None
        self.goal_pose = None
        self.occupancy_grid = None
        self.current_path = None
        self.temp_goal_queue = []
        self.current_temp_goal_index = 0
        self.is_navigating = False
        
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
        
        self.path_publisher = self.create_publisher(
            Path,
            '/planned_path',
            10
        )
        
        self.temp_goal_path_publisher = self.create_publisher(
            Path,
            '/temp_goal_path',
            10
        )
        
        # 创建Nav2 ComputePathToPose Action客户端
        self.compute_path_client = ActionClient(
            self,
            ComputePathToPose,
            'compute_path_to_pose'
        )
        
        # 创建定时器
        timer_period = 1.0 / self.replan_frequency
        self.control_timer = self.create_timer(timer_period, self.control_loop)
        
        self.get_logger().info('Nav2 Navigation Node Started')
        self.get_logger().info(f'Planner ID: {self.planner_id}')
        self.get_logger().info(f'Temp goal spacing: {self.temp_goal_spacing}m')
        self.get_logger().info(f'Switch distance: {self.temp_goal_switch_distance}m')
        self.get_logger().info(f'Goal reached distance: {self.goal_reached_distance}m')
        
    def goal_callback(self, msg: PoseStamped):
        """接收新目标点"""
        self.goal_pose = msg
        self.is_navigating = True
        self.current_temp_goal_index = 0
        self.temp_goal_queue = []
        
        self.get_logger().info(
            f'New goal received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})'
        )
        
        # 立即请求路径规划
        self.request_path_planning()
        
    def odom_callback(self, msg: Odometry):
        """接收里程计数据"""
        self.current_pose = msg.pose.pose
        
    def map_callback(self, msg: OccupancyGrid):
        """接收地图数据"""
        self.occupancy_grid = msg
        
    def request_path_planning(self):
        """请求Nav2进行路径规划"""
        if self.current_pose is None or self.goal_pose is None:
            self.get_logger().warn('Cannot plan path: missing pose or goal')
            return
        
        if not self.compute_path_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Nav2 compute_path_to_pose server not available!')
            self.get_logger().error('Please ensure planner_server is running!')
            return
        
        # 记录当前位置和目标位置
        self.get_logger().info(
            f'Planning path from ({self.current_pose.position.x:.2f}, {self.current_pose.position.y:.2f}) '
            f'to ({self.goal_pose.pose.position.x:.2f}, {self.goal_pose.pose.position.y:.2f})'
        )
        
        # 创建目标消息
        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = self.goal_pose
        goal_msg.planner_id = self.planner_id
        goal_msg.use_start = False  # 使用当前机器人位置作为起点
        
        self.get_logger().info(f'Requesting path planning with {self.planner_id}...')
        
        # 异步发送目标
        send_goal_future = self.compute_path_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        """处理路径规划请求响应"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Path planning goal rejected by Nav2')
            return
        
        self.get_logger().info('Path planning goal accepted by Nav2')
        
        # 获取结果
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.path_result_callback)
        
    def path_result_callback(self, future):
        """处理路径规划结果"""
        try:
            result = future.result().result
        except Exception as e:
            self.get_logger().error(f'Failed to get path planning result: {str(e)}')
            self.is_navigating = False
            return
        
        if not result or not result.path or len(result.path.poses) < 2:
            self.get_logger().error('Nav2 returned empty or invalid path')
            self.get_logger().error('Possible reasons:')
            self.get_logger().error('  1. Start or goal position is inside an obstacle')
            self.get_logger().error('  2. No valid path exists (obstacles blocking)')
            self.get_logger().error('  3. Goal is outside the costmap bounds')
            self.get_logger().error('Suggestions:')
            self.get_logger().error('  - Check if goal is in free space (not on obstacle)')
            self.get_logger().error('  - Verify costmap covers the goal area')
            self.get_logger().error('  - Try setting goal closer to current position')
            self.get_logger().error('  - Check inflation_radius in nav2_params.yaml')
            
            # 生成简单的直线路径作为回退方案
            self.get_logger().warn('Attempting fallback: direct line to goal...')
            fallback_path = self.create_simple_fallback_path()
            if fallback_path:
                self.extract_temp_goals_from_path(fallback_path)
            else:
                self.is_navigating = False
            return
        
        self.current_path = result.path
        path_length = len(result.path.poses)
        
        self.get_logger().info(f'Received path from Nav2: {path_length} points')
        
        # 发布完整路径用于可视化
        self.path_publisher.publish(result.path)
        
        # 从路径中提取临时目标点
        self.extract_temp_goals_from_path(result.path)
        
    def extract_temp_goals_from_path(self, path: Path):
        """从Nav2规划的路径中提取临时目标点"""
        if not path or not path.poses or len(path.poses) < 2:
            self.get_logger().warn('Path too short to extract temp goals')
            return
        
        temp_goals = []
        accumulated_distance = 0.0
        
        # 添加第一个点（起点附近）
        if len(path.poses) > 0:
            first_pose = path.poses[0]
            temp_goals.append((
                first_pose.pose.position.x,
                first_pose.pose.position.y
            ))
        
        # 按间距采样路径点
        for i in range(1, len(path.poses)):
            prev_x = path.poses[i-1].pose.position.x
            prev_y = path.poses[i-1].pose.position.y
            curr_x = path.poses[i].pose.position.x
            curr_y = path.poses[i].pose.position.y
            
            segment_length = math.hypot(curr_x - prev_x, curr_y - prev_y)
            accumulated_distance += segment_length
            
            # 当累积距离超过间距时，添加临时目标点
            if accumulated_distance >= self.temp_goal_spacing:
                temp_goals.append((curr_x, curr_y))
                accumulated_distance = 0.0
        
        # 添加最终目标点
        if len(path.poses) > 0:
            last_pose = path.poses[-1]
            last_point = (last_pose.pose.position.x, last_pose.pose.position.y)
            
            # 如果最后一个临时目标点离终点较远，添加终点
            if not temp_goals or math.hypot(
                last_point[0] - temp_goals[-1][0],
                last_point[1] - temp_goals[-1][1]
            ) > self.temp_goal_spacing * 0.3:
                temp_goals.append(last_point)
        
        self.temp_goal_queue = temp_goals
        self.current_temp_goal_index = 0
        
        self.get_logger().info(f'Extracted {len(temp_goals)} temporary goals from path')
        
        # 发布临时目标点路径用于可视化
        self.publish_temp_goal_path()
        
    def publish_temp_goal_path(self):
        """发布临时目标点路径用于可视化"""
        if not self.temp_goal_queue:
            return
        
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for x, y in self.temp_goal_queue:
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.w = 1.0
            path_msg.poses.append(pose_stamped)
        
        self.temp_goal_path_publisher.publish(path_msg)
        
    def control_loop(self):
        """主控制循环"""
        if not self.is_navigating:
            return
        
        if self.current_pose is None or self.goal_pose is None:
            return
        
        # 检查是否到达最终目标
        goal_distance = self.distance_to_goal()
        if goal_distance < self.goal_reached_distance:
            self.get_logger().info(f'Reached final goal! Distance: {goal_distance:.2f}m')
            self.is_navigating = False
            return
        
        # 如果没有临时目标点，尝试重新规划
        if not self.temp_goal_queue:
            self.get_logger().warn('No temp goals available, requesting new path...')
            self.request_path_planning()
            return
        
        # 检查当前临时目标点
        if self.current_temp_goal_index >= len(self.temp_goal_queue):
            self.get_logger().warn('Reached end of temp goal queue, requesting new path...')
            self.request_path_planning()
            return
        
        # 获取当前临时目标点
        temp_goal_x, temp_goal_y = self.temp_goal_queue[self.current_temp_goal_index]
        
        # 计算到当前临时目标的距离
        distance_to_temp_goal = math.hypot(
            temp_goal_x - self.current_pose.position.x,
            temp_goal_y - self.current_pose.position.y
        )
        
        # 如果接近当前临时目标，切换到下一个
        if distance_to_temp_goal < self.temp_goal_switch_distance:
            self.current_temp_goal_index += 1
            
            if self.current_temp_goal_index < len(self.temp_goal_queue):
                temp_goal_x, temp_goal_y = self.temp_goal_queue[self.current_temp_goal_index]
                self.get_logger().info(
                    f'Switching to temp goal {self.current_temp_goal_index}/{len(self.temp_goal_queue)}: '
                    f'({temp_goal_x:.2f}, {temp_goal_y:.2f})'
                )
            else:
                # 到达队列末尾，重新规划
                self.get_logger().info('Reached end of temp goal queue, replanning...')
                self.request_path_planning()
                return
        
        # 发布当前临时目标点
        self.publish_temp_goal(temp_goal_x, temp_goal_y)
        
    def distance_to_goal(self) -> float:
        """计算到最终目标的距离"""
        if self.current_pose is None or self.goal_pose is None:
            return float('inf')
        
        dx = self.goal_pose.pose.position.x - self.current_pose.position.x
        dy = self.goal_pose.pose.position.y - self.current_pose.position.y
        return math.hypot(dx, dy)
        
    def publish_temp_goal(self, x: float, y: float):
        """发布临时目标点"""
        temp_goal_msg = Point()
        temp_goal_msg.x = x
        temp_goal_msg.y = y
        temp_goal_msg.z = 0.0
        
        self.temp_goal_publisher.publish(temp_goal_msg)
    
    def create_simple_fallback_path(self) -> Optional[Path]:
        """创建简单的回退路径（直线）"""
        if self.current_pose is None or self.goal_pose is None:
            return None
        
        self.get_logger().warn('Creating simple fallback path (direct line)')
        
        start_x = self.current_pose.position.x
        start_y = self.current_pose.position.y
        goal_x = self.goal_pose.pose.position.x
        goal_y = self.goal_pose.pose.position.y
        
        # 计算总距离
        total_distance = math.hypot(goal_x - start_x, goal_y - start_y)
        
        if total_distance < 0.1:
            self.get_logger().warn('Goal too close to start, no path needed')
            return None
        
        # 创建路径消息
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        
        # 按固定间距插值路径点
        num_points = max(int(total_distance / 0.5), 2)  # 每0.5米一个点
        
        for i in range(num_points + 1):
            t = i / num_points
            
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = start_x + t * (goal_x - start_x)
            pose.pose.position.y = start_y + t * (goal_y - start_y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            
            path.poses.append(pose)
        
        self.get_logger().warn(f'Fallback path created with {len(path.poses)} points')
        self.get_logger().warn('WARNING: This path does NOT avoid obstacles!')
        
        # 发布路径用于可视化
        self.path_publisher.publish(path)
        
        return path


def main(args=None):
    rclpy.init(args=args)
    
    node = Nav2NavigationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
