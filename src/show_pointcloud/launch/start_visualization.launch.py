#!/usr/bin/env python3
"""
启动文件：启动点云可视化服务（HTTP轮询版）
包括：Flask HTTP API服务器 + 静态文件服务器
适用于遥控器系统和WebSocket不兼容的环境
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
        LogInfo(msg='========================================'),
        LogInfo(msg='Starting FAST-LIO PointCloud Visualization (HTTP Polling)'),
        LogInfo(msg='========================================'),
        LogInfo(msg=f'Web files directory: {web_dir}'),
        
        # 启动Flask HTTP API服务器（端口9001）
        Node(
            package='show_pointcloud',
            executable='pointcloud_flask_server',
            name='pointcloud_http_api_server',
            output='screen',
            parameters=[{
                'http_port': 9001,
                'max_points': 50000,
                'downsample_factor': 5,
                'update_rate': 10.0,
                'enable_full_cloud': False
            }]
        ),
        
        # 启动静态文件HTTP服务器（端口8002）
        ExecuteProcess(
            cmd=['python3', '-m', 'http.server', '8002'],
            cwd=web_dir,
            output='screen',
            shell=False
        ),
        
        LogInfo(msg=''),
        LogInfo(msg='========================================'),
        LogInfo(msg='✅ PointCloud Visualization Servers Started!'),
        LogInfo(msg='========================================'),
        LogInfo(msg='📊 HTTP API服务器: http://<your-ip>:9001/api'),
        LogInfo(msg='   - GET /api/config  - 获取配置'),
        LogInfo(msg='   - GET /api/status  - 获取状态'),
        LogInfo(msg='   - GET /api/data    - 获取点云数据'),
        LogInfo(msg=''),
        LogInfo(msg='🌐 Web界面:'),
        LogInfo(msg='   - http://<your-ip>:8002/http_polling_3d.html (3D可视化)'),
        LogInfo(msg='   - http://<your-ip>:8002/debug_polling.html (调试版)'),
        LogInfo(msg='   - http://<your-ip>:8002/ultimate_test.html (API测试)'),
        LogInfo(msg=''),
        LogInfo(msg='📱 适用于：'),
        LogInfo(msg='   - 安卓手机浏览器'),
        LogInfo(msg='   - 遥控器系统转发'),
        LogInfo(msg='   - WebSocket不兼容环境'),
        LogInfo(msg='========================================'),
    ])
