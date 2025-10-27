#!/bin/bash

echo "======================================="
echo "停止旧进程..."
echo "======================================="

# 停止旧的进程
pkill -f "pointcloud_server" 2>/dev/null
pkill -f "http.server.*8000" 2>/dev/null

sleep 2

echo ""
echo "======================================="
echo "启动 FAST-LIO 点云可视化服务"
echo "======================================="

# 获取本机IP
IP=$(hostname -I | awk '{print $1}')

echo ""
echo "📡 本机IP地址: $IP"
echo "🌐 访问地址: http://$IP:8000"
echo "🔌 WebSocket端口: 9000"
echo ""
echo "正在启动服务..."
echo ""

# Source ROS2环境
cd /home/bestway/bestway
source install/setup.bash

# 启动launch文件
ros2 launch show_pointcloud start_visualization.launch.py
