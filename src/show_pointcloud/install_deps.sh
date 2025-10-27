#!/bin/bash

echo "======================================="
echo "安装 PointCloud 可视化依赖..."
echo "======================================="

# 安装websockets库
echo "正在安装 websockets..."
pip3 install websockets

# 检查安装
echo ""
echo "检查依赖安装..."
python3 -c "import websockets; print('✓ websockets 安装成功')" || echo "✗ websockets 安装失败"
python3 -c "import rclpy; print('✓ rclpy 已安装')" || echo "✗ rclpy 未安装"

echo ""
echo "======================================="
echo "依赖安装完成！"
echo "======================================="
echo ""
echo "使用方法："
echo "1. 启动服务："
echo "   ros2 launch show_pointcloud start_visualization.launch.py"
echo ""
echo "2. 在Android浏览器打开："
echo "   http://<你的IP>:8000"
echo ""
echo "查看本机IP："
echo "   hostname -I"
echo "======================================="
