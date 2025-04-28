# Bestway 项目

## 项目简介
Bestway 是一个基于 ROS 2 的机器人控制项目，包含以下功能：
- **键盘控制节点**：通过键盘控制机器人移动。
- **SBUS 控制节点**：将控制指令转换为 SBUS 数据并通过串口发送。

---

## 功能特性
1. **键盘控制**：
   - 使用 `w`, `s`, `a`, `d` 控制机器人前进、后退、左转、右转。
   - 使用 `i`, `j`, `o`, `k` 调整线速度和角速度。
   - 松开键盘后，机器人会自动停止。

2. **SBUS 数据发送**：
   - 将 `/cmd_vel` 消息转换为 SBUS 数据帧。
   - 通过串口发送控制指令。

---

## 环境要求
- **操作系统**：Ubuntu 20.04 或更高版本
- **ROS 2**：Foxy、Galactic 或 Humble
- **Python**：3.8 或更高版本
- **依赖库**：
  - `pyserial`：用于串口通信

---

## 安装与运行

### 1. 克隆项目
```bash
git clone https://github.com/HeYuJie2019/bestway.git
cd bestway
```

### 2. 安装依赖
```bash
pip install pyserial
```

### 3. 构建项目
```bash
colcon build
source [setup.bash](http://_vscodecontentref_/0)
```

### 4. 运行键盘控制节点
```bash
ros2 run sbus_control keyboard_control_node
```

### 5. 运行 SBUS 控制节点
```bash
ros2 run sbus_control sbus_control_node
```