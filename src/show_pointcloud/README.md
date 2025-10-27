# Show PointCloud - FAST-LIO 实时点云Web可视化

这个ROS2包用于在Android设备（或任何带浏览器的设备）上实时显示FAST-LIO算法构建的点云地图。

## 功能特性

- 📡 **实时显示**：通过WebSocket实时推送点云数据
- 📱 **移动友好**：支持Android设备和触摸操作
- 🎨 **多种着色模式**：高度着色、距离着色、单色
- 🔄 **自动下采样**：优化性能，保证流畅显示
- 📊 **实时统计**：显示点云数量、FPS、位置信息
- 🌐 **零配置**：自动启动HTTP和WebSocket服务器

## 系统架构

```
┌─────────────────┐      ┌──────────────────┐      ┌─────────────────┐
│   FAST-LIO      │─────▶│  ROS2 Node       │─────▶│   WebSocket     │
│   /cloud_       │      │  (Python)        │      │   Server        │
│   registered    │      │  - 订阅点云      │      │   :9000         │
│   /Odometry     │      │  - 下采样        │      └────────┬────────┘
│   /Laser_map    │      │  - 转换格式      │               │
└─────────────────┘      └──────────────────┘               │
                                                             │
                         ┌──────────────────┐               │
                         │  HTTP Server     │◀──────────────┘
                         │  :8000           │
                         │  - index.html    │
                         │  - Three.js      │
                         └────────┬─────────┘
                                  │
                         ┌────────▼─────────┐
                         │  Android Browser │
                         │  实时3D显示      │
                         └──────────────────┘
```

## 安装依赖

```bash
# 安装Python依赖
pip3 install websockets

# 或者使用系统包管理器
sudo apt-get install python3-websockets
```

## 编译

```bash
cd ~/bestway
colcon build --packages-select show_pointcloud
source install/setup.bash
```

## 使用方法

### 1. 启动可视化服务

```bash
ros2 launch show_pointcloud start_visualization.launch.py
```

这会同时启动：
- ROS2节点（订阅FAST-LIO话题）
- WebSocket服务器（端口9000）
- HTTP服务器（端口8000）

### 2. 在Android设备上打开浏览器

1. 确保Android设备和电脑在同一网络
2. 查看电脑IP地址：`hostname -I`
3. 在Android浏览器中打开：`http://你的IP:8000`

例如：`http://192.168.1.100:8000`

### 3. 控制说明

- **触摸拖动**：旋转视角
- **双指缩放**：放大/缩小
- **重置视角**：点击"重置视角"按钮
- **切换颜色**：切换不同的点云着色模式
- **清除地图**：清空当前显示的点云

## 参数配置

可以通过修改launch文件中的参数来调整性能：

```python
parameters=[{
    'websocket_port': 9000,        # WebSocket端口
    'max_points': 50000,           # 最大显示点数
    'downsample_factor': 5,        # 下采样因子（每5个点取1个）
    'update_rate': 10.0            # 更新频率（Hz）
}]
```

### 性能优化建议

- **网络较慢**：增大`downsample_factor`到10，减少`max_points`到30000
- **设备性能差**：降低`update_rate`到5.0，减少点云数量
- **需要更高精度**：减小`downsample_factor`到2-3，但需要更好的网络

## 订阅的话题

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/cloud_registered` | sensor_msgs/PointCloud2 | 当前帧配准后的点云 |
| `/Odometry` | nav_msgs/Odometry | 里程计信息（位置和姿态） |
| `/Laser_map` | sensor_msgs/PointCloud2 | 累积的地图点云 |

## 故障排除

### 1. 无法连接WebSocket

- 检查防火墙是否开放9000端口
- 确认ROS2节点正常运行：`ros2 node list`
- 查看节点日志：`ros2 node info /pointcloud_web_server`

### 2. HTTP服务器无法访问

- 检查8000端口是否被占用：`lsof -i :8000`
- 确认web文件已正确安装：`ros2 pkg prefix show_pointcloud`

### 3. 点云显示卡顿

- 增大下采样因子
- 减少最大点数
- 降低更新频率
- 检查网络延迟

### 4. 没有数据显示

- 确认FAST-LIO节点正在运行
- 检查话题是否发布：`ros2 topic list`
- 查看话题频率：`ros2 topic hz /cloud_registered`

## 开发和扩展

### 添加新的着色模式

在`index.html`的`getPointColor`函数中添加新的颜色计算逻辑。

### 调整显示效果

修改Three.js的材质参数：
```javascript
const material = new THREE.PointsMaterial({
    size: 0.05,        // 点的大小
    vertexColors: true,
    sizeAttenuation: true  // 距离衰减
});
```

### 添加更多传感器数据

在`pointcloud_server.py`中添加新的订阅器，并在WebSocket消息中包含新数据。

## 技术栈

- **后端**：ROS2 (Python), WebSocket
- **前端**：Three.js, HTML5, JavaScript
- **通信**：WebSocket (实时双向通信)
- **服务器**：Python HTTP Server

## 许可证

Apache-2.0

## 作者

FAST-LIO点云可视化系统

## 更新日志

### v0.0.1 (2025-10-27)
- 初始版本
- 基本点云显示功能
- WebSocket实时通信
- 移动端支持
