#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header
import numpy as np
import struct
import math
import time
from typing import Tuple, List, Optional


class PointCloudTo2DMap(Node):
    """
    将三维点云数据转换为二维栅格地图的ROS2节点
    用于将fast_lio2的三维SLAM结果转换为可用于导航的二维地图
    """
    
    def __init__(self):
        super().__init__('pointcloud_to_2d_map')
        
        # 声明参数
        # self.declare_parameter('input_topic', '/cloud_registered')
        self.declare_parameter('input_topic', '/Laser_map')
        self.declare_parameter('output_topic', '/map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('map_resolution', 0.05)  # 5cm分辨率
        self.declare_parameter('map_width', 2000)       # 地图宽度(像素)
        self.declare_parameter('map_height', 2000)      # 地图高度(像素)
        self.declare_parameter('obstacle_z_min', -0.3)   # 障碍物最小高度
        self.declare_parameter('obstacle_z_max', 1.0)   # 障碍物最大高度
        self.declare_parameter('update_rate', 50.0)      # 降低更新频率减少闪烁
        self.declare_parameter('inflation_radius', 0.15) # 膨胀半径
        self.declare_parameter('fast_mode', True)        # 快速模式，简化处理
        self.declare_parameter('skip_frames', 3)         # 增加跳帧减少更新
        
        # 获取参数
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.map_resolution = self.get_parameter('map_resolution').value
        self.map_width = self.get_parameter('map_width').value
        self.map_height = self.get_parameter('map_height').value
        self.obstacle_z_min = self.get_parameter('obstacle_z_min').value
        self.obstacle_z_max = self.get_parameter('obstacle_z_max').value
        self.update_rate = self.get_parameter('update_rate').value
        self.inflation_radius = self.get_parameter('inflation_radius').value
        self.fast_mode = self.get_parameter('fast_mode').value
        self.skip_frames = self.get_parameter('skip_frames').value
        
        # 初始化地图数据 - 使用累积式更新避免闪烁
        self.occupancy_grid = np.full((self.map_height, self.map_width), -1, dtype=np.int8)
        self.hit_count = np.zeros((self.map_height, self.map_width), dtype=np.int32)
        self.miss_count = np.zeros((self.map_height, self.map_width), dtype=np.int32)
        
        # 地图原点(地图中心对应世界坐标原点)
        self.map_origin_x = -self.map_width * self.map_resolution / 2.0
        self.map_origin_y = -self.map_height * self.map_resolution / 2.0
        
        # 最新点云数据和帧计数
        self.latest_pointcloud = None
        self.last_update_time = time.time()
        self.frame_count = 0
        
        # 创建订阅者和发布者
        self.pointcloud_subscriber = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.pointcloud_callback,
            10
        )
        
        # 使用transient_local QoS以便Nav2的static_layer能正确订阅
        from rclpy.qos import QoSProfile, DurabilityPolicy
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        
        self.map_publisher = self.create_publisher(
            OccupancyGrid,
            self.output_topic,
            map_qos
        )
        
        # 创建定时器进行地图更新
        timer_period = 1.0 / self.update_rate
        self.timer = self.create_timer(timer_period, self.update_map)
        
        self.get_logger().info(f'PointCloud to 2D Map converter started - Fast Mode')
        self.get_logger().info(f'Input topic: {self.input_topic}')
        self.get_logger().info(f'Output topic: {self.output_topic}')
        self.get_logger().info(f'Map resolution: {self.map_resolution}m')
        self.get_logger().info(f'Map size: {self.map_width}x{self.map_height}')
        self.get_logger().info(f'Height range: {self.obstacle_z_min}m to {self.obstacle_z_max}m')
        self.get_logger().info(f'Update rate: {self.update_rate}Hz')
        self.get_logger().info(f'Fast mode: {self.fast_mode}')
        
    def pointcloud_callback(self, msg: PointCloud2):
        """点云数据回调函数"""
        self.frame_count += 1
        
        # 跳帧处理以提高性能
        if self.frame_count % self.skip_frames != 0:
            return
            
        self.latest_pointcloud = msg
        
    def parse_pointcloud_fast(self, pointcloud_msg: PointCloud2) -> np.ndarray:
        """快速解析点云数据，直接过滤高度并返回xy坐标"""
        try:
            # 获取点云字段信息
            fields = {}
            for field in pointcloud_msg.fields:
                fields[field.name] = field
            
            # 确保有xyz字段
            if 'x' not in fields or 'y' not in fields or 'z' not in fields:
                self.get_logger().warn('PointCloud missing x, y, or z fields')
                return np.array([])
            
            # 获取字段偏移
            x_offset = fields['x'].offset
            y_offset = fields['y'].offset
            z_offset = fields['z'].offset
            
            # 使用numpy进行批量处理
            point_step = pointcloud_msg.point_step
            data = pointcloud_msg.data
            num_points = len(data) // point_step
            
            if num_points == 0:
                return np.array([])
            
            # 创建结构化数组来快速解析数据
            dt = np.dtype([
                ('x', np.float32),
                ('y', np.float32), 
                ('z', np.float32)
            ])
            
            # 预分配数组
            points_xy = []
            
            # 批量处理点云数据
            for i in range(0, len(data), point_step):
                if i + point_step <= len(data):
                    try:
                        x = struct.unpack('f', data[i + x_offset:i + x_offset + 4])[0]
                        y = struct.unpack('f', data[i + y_offset:i + y_offset + 4])[0]
                        z = struct.unpack('f', data[i + z_offset:i + z_offset + 4])[0]
                        
                        # 快速过滤：只保留指定高度范围内的点
                        if (not (math.isnan(x) or math.isnan(y) or math.isnan(z)) and 
                            self.obstacle_z_min <= z <= self.obstacle_z_max):
                            points_xy.append([x, y])
                    except:
                        continue
            
            return np.array(points_xy)
            
        except Exception as e:
            self.get_logger().error(f'Error parsing pointcloud: {str(e)}')
            return np.array([])
    
    def world_to_map(self, world_x: float, world_y: float) -> Tuple[int, int]:
        """世界坐标转地图坐标"""
        map_x = int((world_x - self.map_origin_x) / self.map_resolution)
        map_y = int((world_y - self.map_origin_y) / self.map_resolution)
        return map_x, map_y
    
    def is_valid_map_coord(self, map_x: int, map_y: int) -> bool:
        """检查地图坐标是否有效"""
        return 0 <= map_x < self.map_width and 0 <= map_y < self.map_height
    
    def update_occupancy_grid_fast(self, points_xy: np.ndarray):
        """累积式更新占用栅格 - 避免闪烁"""
        if len(points_xy) == 0:
            return
        
        # 批量转换世界坐标到地图坐标
        map_coords = np.zeros((len(points_xy), 2), dtype=np.int32)
        map_coords[:, 0] = ((points_xy[:, 0] - self.map_origin_x) / self.map_resolution).astype(np.int32)
        map_coords[:, 1] = ((points_xy[:, 1] - self.map_origin_y) / self.map_resolution).astype(np.int32)
        
        # 过滤有效坐标
        valid_mask = ((map_coords[:, 0] >= 0) & (map_coords[:, 0] < self.map_width) & 
                     (map_coords[:, 1] >= 0) & (map_coords[:, 1] < self.map_height))
        
        valid_coords = map_coords[valid_mask]
        
        if len(valid_coords) > 0:
            # 累积式更新：增加hit计数
            for coord in valid_coords:
                self.hit_count[coord[1], coord[0]] += 1
        
        # 计算占用概率并更新地图
        self.compute_occupancy_probabilities()
    
    def compute_occupancy_probabilities(self):
        """计算占用概率 - 简化版本避免闪烁"""
        total_count = self.hit_count + self.miss_count
        
        # 避免除零
        valid_mask = total_count > 0
        
        # 计算有效区域的占用概率
        hit_prob = np.zeros_like(total_count, dtype=np.float32)
        hit_prob[valid_mask] = self.hit_count[valid_mask] / total_count[valid_mask]
        
        # 设置占用状态
        # 高hit率 -> 占用(100)
        self.occupancy_grid[hit_prob > 0.6] = 100
        
        # 低hit率 -> 空闲(0) 
        self.occupancy_grid[(hit_prob <= 0.6) & (hit_prob > 0) & valid_mask] = 0
        
        # 数据不足的区域保持未知(-1)
        self.occupancy_grid[total_count < 3] = -1
    
    def inflate_obstacles_fast(self, occupancy: np.ndarray) -> np.ndarray:
        """快速膨胀障碍物"""
        if self.inflation_radius <= 0:
            return occupancy
        
        inflated = occupancy.copy()
        inflation_pixels = int(self.inflation_radius / self.map_resolution)
        
        if inflation_pixels <= 0:
            return occupancy
        
        # 找到所有障碍物像素
        obstacle_y, obstacle_x = np.where(occupancy == 100)
        
        # 为每个障碍物创建膨胀区域
        for i in range(len(obstacle_y)):
            y, x = obstacle_y[i], obstacle_x[i]
            
            # 计算膨胀区域边界
            y_min = max(0, y - inflation_pixels)
            y_max = min(self.map_height, y + inflation_pixels + 1)
            x_min = max(0, x - inflation_pixels)
            x_max = min(self.map_width, x + inflation_pixels + 1)
            
            # 在膨胀区域内设置为占用（使用较低的值以区分原始障碍物）
            for dy in range(y_min, y_max):
                for dx in range(x_min, x_max):
                    distance_sq = (dy - y)**2 + (dx - x)**2
                    if distance_sq <= inflation_pixels**2:
                        if inflated[dy, dx] < 100:  # 不覆盖原始障碍物
                            inflated[dy, dx] = 80
        
        return inflated
    
    def world_to_map(self, world_x: float, world_y: float) -> Tuple[int, int]:
        """世界坐标转地图坐标"""
        map_x = int((world_x - self.map_origin_x) / self.map_resolution)
        map_y = int((world_y - self.map_origin_y) / self.map_resolution)
        return map_x, map_y
    
    def is_valid_map_coord(self, map_x: int, map_y: int) -> bool:
        """检查地图坐标是否有效"""
        return 0 <= map_x < self.map_width and 0 <= map_y < self.map_height
    
    def update_map(self):
        """快速更新地图"""
        if self.latest_pointcloud is None:
            return
        
        current_time = time.time()
        if current_time - self.last_update_time < (1.0 / self.update_rate):
            return
        
        start_time = time.time()
        
        # 快速解析点云 - 直接获取过滤后的XY坐标
        points_xy = self.parse_pointcloud_fast(self.latest_pointcloud)
        
        parse_time = time.time()
        
        if len(points_xy) == 0:
            return
        
        # 快速更新占用栅格 - 累积式更新避免闪烁
        self.update_occupancy_grid_fast(points_xy)
        
        update_time = time.time()
        
        # 快速膨胀障碍物
        if self.inflation_radius > 0:
            self.occupancy_grid = self.inflate_obstacles_fast(self.occupancy_grid)
        
        inflate_time = time.time()
        
        # 发布地图
        self.publish_map_fast(len(points_xy))
        
        publish_time = time.time()
        
        # 性能统计
        total_time = publish_time - start_time
        if total_time > 0.02:  # 如果处理时间超过20ms，输出性能信息
            self.get_logger().info(
                f'Map update time: {total_time*1000:.1f}ms '
                f'(parse: {(parse_time-start_time)*1000:.1f}ms, '
                f'update: {(update_time-parse_time)*1000:.1f}ms, '
                f'inflate: {(inflate_time-update_time)*1000:.1f}ms, '
                f'publish: {(publish_time-inflate_time)*1000:.1f}ms)'
            )
        
        self.last_update_time = current_time
    
    def publish_map_fast(self, point_count: int):
        """快速发布占用栅格地图"""
        map_msg = OccupancyGrid()
        map_msg.header = Header()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = self.map_frame
        
        # 设置地图元数据
        map_msg.info = MapMetaData()
        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.map_width
        map_msg.info.height = self.map_height
        
        # 设置地图原点
        map_msg.info.origin = Pose()
        map_msg.info.origin.position = Point()
        map_msg.info.origin.position.x = self.map_origin_x
        map_msg.info.origin.position.y = self.map_origin_y
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation = Quaternion()
        map_msg.info.origin.orientation.w = 1.0
        
        # 设置地图数据(行优先，从左下角开始)
        map_msg.data = self.occupancy_grid.flatten().tolist()
        
        self.map_publisher.publish(map_msg)
        
        # 简化的统计信息 - 减少频繁输出
        free_count = np.sum(self.occupancy_grid == 0)
        occupied_count = np.sum(self.occupancy_grid >= 80)
        
        # 降低日志输出频率
        if hasattr(self, 'log_counter'):
            self.log_counter += 1
        else:
            self.log_counter = 0
            
        if self.log_counter % 10 == 0:  # 每10次更新输出一次日志
            self.get_logger().info(
                f'Map: {free_count} free, {occupied_count} occupied | {point_count} points'
            )
    

def main(args=None):
    rclpy.init(args=args)
    
    node = PointCloudTo2DMap()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
