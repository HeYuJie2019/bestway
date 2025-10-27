#!/usr/bin/env python3
"""
ROS2节点：订阅FAST-LIO点云数据并通过WebSocket实时推送到Web浏览器
支持在Android设备上实时显示点云构建过程
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import asyncio
import websockets
import json
import threading
import struct
import time
from collections import deque


class PointCloudWebServer(Node):
    def __init__(self):
        super().__init__('pointcloud_web_server')
        
        # 声明参数
        self.declare_parameter('websocket_port', 9000)
        self.declare_parameter('max_points', 100000)         # 点云限制
        self.declare_parameter('downsample_factor', 5)       # 下采样因子（更小=更多点）
        self.declare_parameter('enable_full_cloud', True)   # 是否显示全部点云（无下采样）
        self.declare_parameter('update_rate', 10.0)          # Hz
        
        self.ws_port = self.get_parameter('websocket_port').value
        self.max_points = self.get_parameter('max_points').value
        self.downsample = self.get_parameter('downsample_factor').value
        self.enable_full_cloud = self.get_parameter('enable_full_cloud').value
        self.update_rate = self.get_parameter('update_rate').value
        
        # 数据缓存
        self.latest_pointcloud = None     # 点云数据
        self.latest_odom = None
        self.pointcloud_lock = threading.Lock()
        self.connected_clients = set()
        
        # 只订阅当前帧点云
        self.create_subscription(
            PointCloud2,
            # '/cloud_registered',
            '/Laser_map',
            self.pointcloud_callback,
            10
        )
        
        self.create_subscription(
            Odometry,
            '/Odometry',
            self.odom_callback,
            10
        )
        
        self.get_logger().info(f'PointCloud Web Server initialized')
        self.get_logger().info(f'WebSocket server will run on port {self.ws_port}')
        self.get_logger().info(f'Max points: {self.max_points}, Downsample: {self.downsample}')
        self.get_logger().info(f'Full cloud mode: {"ENABLED" if self.enable_full_cloud else "DISABLED"}')
        self.get_logger().info(f'Subscribing to /Laser_map topic')
        
        # 启动WebSocket服务器
        self.ws_thread = threading.Thread(target=self.run_websocket_server, daemon=True)
        self.ws_thread.start()
        
        # 统计信息
        self.msg_count = 0
        self.last_log_time = time.time()
    
    def pointcloud_callback(self, msg):
        """处理点云数据 - 直接显示Laser_map"""
        try:
            # 提取点云数据
            points = []
            for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                # 确保转换为标准Python float类型
                points.append([float(point[0]), float(point[1]), float(point[2])])
            
            if len(points) == 0:
                return
            
            original_count = len(points)
            
            # 根据参数决定是否下采样
            if not self.enable_full_cloud:
                # 下采样模式
                points = points[::self.downsample]
            
            # 限制点数（防止内存溢出）
            if len(points) > self.max_points:
                indices = np.random.choice(len(points), self.max_points, replace=False)
                points = [points[i] for i in indices]
            
            timestamp = float(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
            frame_id = str(msg.header.frame_id)
            
            with self.pointcloud_lock:
                self.latest_pointcloud = {
                    'points': points,
                    'timestamp': timestamp,
                    'frame_id': frame_id
                }
            
            self.msg_count += 1
            
            # 每秒打印一次统计信息
            current_time = time.time()
            if current_time - self.last_log_time > 1.0:
                mode_str = "FULL" if self.enable_full_cloud else f"SAMPLED(1/{self.downsample})"
                self.get_logger().info(
                    f'Laser_map: {self.msg_count} msgs, {original_count}→{len(points)} points ({mode_str}), '
                    f'{len(self.connected_clients)} clients'
                )
                self.msg_count = 0
                self.last_log_time = current_time
                
        except Exception as e:
            self.get_logger().error(f'Error in pointcloud_callback: {str(e)}')

    
    def odom_callback(self, msg):
        """处理里程计数据"""
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
    

    
    def run_websocket_server(self):
        """运行WebSocket服务器"""
        async def handler(websocket):
            client_address = f"{websocket.remote_address[0]}:{websocket.remote_address[1]}"
            self.connected_clients.add(websocket)
            self.get_logger().info(f'New client connected from {client_address}. Total clients: {len(self.connected_clients)}')
            
            try:
                # 发送初始配置
                config_message = json.dumps({
                    'type': 'config',
                    'max_points': self.max_points,
                    'update_rate': self.update_rate,
                    'enable_full_cloud': self.enable_full_cloud,
                    'downsample_factor': self.downsample
                })
                await websocket.send(config_message)
                self.get_logger().info(f'Sent config to {client_address}')
                
                # 持续发送数据
                while True:
                    try:
                        # 检查连接状态
                        if websocket.close_code is not None:
                            break
                            
                        with self.pointcloud_lock:
                            data = {
                                'type': 'update',
                                'pointcloud': self.latest_pointcloud,
                                'odom': self.latest_odom
                            }
                        
                        if self.latest_pointcloud is not None:
                            # 确保数据可以序列化
                            json_data = json.dumps(data)
                            await websocket.send(json_data)
                    except (TypeError, ValueError) as e:
                        self.get_logger().error(f'JSON serialization error: {str(e)}')
                    except websockets.exceptions.ConnectionClosed:
                        self.get_logger().info(f'Client {client_address} connection closed during send')
                        break
                    except Exception as e:
                        self.get_logger().error(f'Error sending data to {client_address}: {str(e)}')
                        break
                    
                    await asyncio.sleep(1.0 / self.update_rate)
                    
            except websockets.exceptions.ConnectionClosed:
                self.get_logger().info(f'Client {client_address} disconnected')
            except websockets.exceptions.InvalidMessage:
                self.get_logger().error(f'Invalid WebSocket message from {client_address}')
            except websockets.exceptions.InvalidHeader:
                self.get_logger().error(f'Invalid WebSocket header from {client_address}')
            except Exception as e:
                self.get_logger().error(f'WebSocket error with {client_address}: {str(e)}')
            finally:
                self.connected_clients.discard(websocket)
                self.get_logger().info(f'Client {client_address} cleaned up')
        
        async def start_server():
            # 添加更多的服务器配置选项以提高兼容性
            server = await websockets.serve(
                handler, 
                '0.0.0.0', 
                self.ws_port,
                # 添加这些选项以提高兼容性
                ping_interval=20,
                ping_timeout=10,
                close_timeout=10,
                max_size=1024*1024*10,  # 10MB max message size
                max_queue=32,
                compression=None  # 禁用压缩以提高兼容性
            )
            self.get_logger().info(f'WebSocket server started on 0.0.0.0:{self.ws_port}')
            self.get_logger().info(f'Server options: ping_interval=20s, max_size=10MB, compression=disabled')
            
            # 等待服务器关闭
            await server.wait_closed()
        
        # 在新线程中运行事件循环
        try:
            asyncio.run(start_server())
        except Exception as e:
            self.get_logger().error(f'Failed to start WebSocket server: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudWebServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
