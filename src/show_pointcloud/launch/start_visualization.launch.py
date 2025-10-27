#!/usr/bin/env python3
"""
启动文件：启动点云可视化服务
包括：ROS2节点 + HTTP服务器
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取包路径
    pkg_dir = get_package_share_directory('show_pointcloud')
    web_dir = os.path.join(pkg_dir, 'web')
    
    return LaunchDescription([
        LogInfo(msg='Starting PointCloud Web Visualization...'),
        LogInfo(msg=f'Web files directory: {web_dir}'),
        
        # 启动ROS2节点（WebSocket服务器）
        Node(
            package='show_pointcloud',
            executable='pointcloud_server',
            name='pointcloud_web_server',
            output='screen',
            parameters=[{
                'websocket_port': 9000,
                'max_points': 50000,
                'downsample_factor': 5,
                'update_rate': 10.0
            }]
        ),
        
        # 启动HTTP服务器
        ExecuteProcess(
            cmd=['python3', '-m', 'http.server', '8000'],
            cwd=web_dir,
            output='screen',
            shell=False
        ),
        
        LogInfo(msg='======================================'),
        LogInfo(msg='PointCloud Visualization Server Started!'),
        LogInfo(msg='Open in browser: http://<your-ip>:8000'),
        LogInfo(msg='WebSocket port: 9000'),
        LogInfo(msg='======================================'),
    ])
