#!/usr/bin/env python3
"""
FAST-LIO点云HTTP服务器 - Flask版本
使用Flask框架提供HTTP REST API服务
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from sensor_msgs_py import point_cloud2
import threading
import time
import json
from flask import Flask, jsonify, request
from flask_cors import CORS

class PointCloudFlaskServer(Node):
    def __init__(self):
        super().__init__('pointcloud_flask_server')
        
        # 声明参数
        self.declare_parameter('max_points', 50000)
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('enable_full_cloud', False)
        self.declare_parameter('downsample', 5)
        self.declare_parameter('http_port', 9001)
        
        # 获取参数
        self.max_points = self.get_parameter('max_points').value
        self.update_rate = self.get_parameter('update_rate').value
        self.enable_full_cloud = self.get_parameter('enable_full_cloud').value
        self.downsample = self.get_parameter('downsample').value
        self.http_port = self.get_parameter('http_port').value
        
        # 数据存储
        self.latest_pointcloud = None
        self.latest_odom = None
        self.pointcloud_lock = threading.Lock()
        self.last_update_time = time.time()
        self.message_count = 0
        self.client_count = 0
        self.total_points_received = 0  # 添加总点数统计
        
        # 创建订阅
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/Laser_map',
            self.pointcloud_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/Odometry',
            self.odom_callback,
            10
        )
        
        # 定时器用于打印状态
        self.create_timer(self.update_rate, self.status_callback)
        
        self.get_logger().info('PointCloud Flask Server initialized')
        self.get_logger().info(f'HTTP API server will run on port {self.http_port}')
        self.get_logger().info(f'Max points: {self.max_points}, Downsample: {self.downsample}')
        self.get_logger().info(f'Full cloud mode: {"ENABLED" if self.enable_full_cloud else "DISABLED"}')
        
        # 启动Flask服务器
        self.run_flask_server()
    
    def pointcloud_callback(self, msg):
        """处理点云消息"""
        try:
            self.message_count += 1
            
            # 读取点云数据
            points = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
            
            total_points = len(points)
            self.total_points_received = total_points  # 更新总点数
            
            # 下采样或限制点数
            if not self.enable_full_cloud:
                points = points[::self.downsample]
            
            if len(points) > self.max_points:
                points = points[:self.max_points]
            
            # 转换为列表格式
            point_list = [[float(p[0]), float(p[1]), float(p[2])] for p in points]
            
            with self.pointcloud_lock:
                self.latest_pointcloud = {
                    'points': point_list,
                    'count': len(point_list),
                    'original_count': total_points
                }
                self.last_update_time = time.time()
                
        except Exception as e:
            self.get_logger().error(f'Error processing pointcloud: {str(e)}')
    
    def odom_callback(self, msg):
        """处理里程计消息"""
        try:
            with self.pointcloud_lock:
                self.latest_odom = {
                    'position': {
                        'x': float(msg.pose.pose.position.x),
                        'y': float(msg.pose.pose.position.y),
                        'z': float(msg.pose.pose.position.z)
                    },
                    'orientation': {
                        'x': float(msg.pose.pose.orientation.x),
                        'y': float(msg.pose.pose.orientation.y),
                        'z': float(msg.pose.pose.orientation.z),
                        'w': float(msg.pose.pose.orientation.w)
                    }
                }
        except Exception as e:
            self.get_logger().error(f'Error processing odometry: {str(e)}')
    
    def status_callback(self):
        """定时打印状态"""
        if self.message_count > 0:
            mode = 'FULL' if self.enable_full_cloud else f'SAMPLED(1/{self.downsample})'
            
            with self.pointcloud_lock:
                display_points = self.latest_pointcloud['count'] if self.latest_pointcloud else 0
            
            # 客户端计数缓慢衰减（每秒衰减10%），这样10Hz的轮询可以保持计数
            self.client_count = max(0, self.client_count * 0.99)
            
            self.get_logger().info(
                f'Flask API: {self.message_count} msgs, {self.total_points_received}→{display_points} points ({mode}), '
                f'{self.client_count:.1f} clients'
            )
    
    def run_flask_server(self):
        """运行Flask服务器"""
        app = Flask(__name__)
        CORS(app)  # 启用CORS
        
        node_ref = self  # 保存对节点的引用
        
        @app.route('/api/config', methods=['GET'])
        def get_config():
            """返回配置信息"""
            node_ref.get_logger().info('📥 Config request received!')
            if node_ref.client_count == 0:
                node_ref.client_count = 0.1
                node_ref.get_logger().info('New client detected (config request)')
            return jsonify({
                'type': 'config',
                'max_points': node_ref.max_points,
                'update_rate': node_ref.update_rate,
                'enable_full_cloud': node_ref.enable_full_cloud,
                'downsample_factor': node_ref.downsample
            })
        
        @app.route('/api/data', methods=['GET'])
        def get_data():
            """返回最新的点云数据"""
            old_count = node_ref.client_count
            node_ref.client_count += 0.1  # 增加客户端计数
            if old_count < 0.5:
                node_ref.get_logger().info(f'📊 Data request! Client count: {old_count:.2f} -> {node_ref.client_count:.2f}')
            with node_ref.pointcloud_lock:
                return jsonify({
                    'type': 'update',
                    'pointcloud': node_ref.latest_pointcloud,
                    'odom': node_ref.latest_odom,
                    'server_time': time.time(),
                    'last_update': node_ref.last_update_time
                })
        
        @app.route('/api/status', methods=['GET'])
        def get_status():
            """返回服务器状态"""
            node_ref.client_count += 0.05  # 增加客户端计数（较小值，因为状态查询较少）
            with node_ref.pointcloud_lock:
                return jsonify({
                    'type': 'status',
                    'server_time': time.time(),
                    'last_update': node_ref.last_update_time,
                    'has_pointcloud': node_ref.latest_pointcloud is not None,
                    'has_odom': node_ref.latest_odom is not None,
                    'point_count': node_ref.latest_pointcloud['count'] if node_ref.latest_pointcloud else 0
                })
        
        # 在单独的线程中运行Flask
        def run_flask():
            node_ref.get_logger().info(f'Flask server starting on 0.0.0.0:{node_ref.http_port}')
            node_ref.get_logger().info('API endpoints:')
            node_ref.get_logger().info('  GET /api/config - 获取配置信息')
            node_ref.get_logger().info('  GET /api/data - 获取最新点云数据')
            node_ref.get_logger().info('  GET /api/status - 获取服务器状态')
            app.run(host='0.0.0.0', port=node_ref.http_port, threaded=True, debug=False)
        
        flask_thread = threading.Thread(target=run_flask, daemon=True)
        flask_thread.start()
        
        # 等待Flask启动
        time.sleep(1)
        node_ref.get_logger().info('Flask server thread started')


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudFlaskServer()
    
    try:
        # 使用spin_once循环
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
