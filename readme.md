# Bestway 项目

## WS2812 红蓝闪烁

本仓库新增了 `src/ws2812_blink.py`，用于控制 WS2812/WS2811（NeoPixel）灯带进行红蓝交替闪烁。

在 Jetson Orin NX 等平台，推荐使用 SPI 方案：`src/ws2812_blink_spi.py`，避免对 PWM/PCM 的依赖。

### 硬件接线（以树莓派为例）
- 数据引脚默认使用 BCM 18（PWM0）。可通过 `--pin` 修改。
- 供电请用 5V 稳定电源，地线 GND 与树莓派 GND 共地。
- 建议在数据线上串联 300–470Ω 电阻，在 5V 与 GND 之间并联 ≥1000µF 电解电容（靠近灯带）。

### 安装依赖
在仓库根目录执行：

```bash
pip install -r requirements.txt
```

部分平台需要以 root 权限运行控制程序：

```bash
sudo -H pip install -r requirements.txt
```

### 运行示例

默认 8 颗灯，GPIO18 引脚，亮度 64，0.5s 红蓝交替：

```bash
sudo python3 src/ws2812_blink.py
```

自定义参数：

```bash
sudo python3 src/ws2812_blink.py \
   --count 30 \
   --pin 18 \
   --brightness 128 \
   --interval 0.2 \
   --loops 100
```

常用参数说明：
- `--count/-n` LED 数量
- `--pin` GPIO（BCM 编号），常见：18(PWM0)、13(PWM1) 等
- `--brightness/-b` 0–255 亮度
- `--interval/-i` 红蓝切换间隔（秒）
- `--loops` 循环次数，0 表示无限
- `--strip-type` 颜色顺序，默认 GRB（适配大多数 WS2812）

### Orin NX（SPI 方案）

启用 SPI：
- 使用 Jetson-io 或相关工具开启 SPI 接口（例如 SPI0），重启后检查 `/dev/spidev0.0` 是否存在。

接线（以 SPI0.0 为例）：
- MOSI -> WS2812 DIN
- GND  -> WS2812 GND（与 5V 电源共地）
运行：
```bash
sudo python3 src/ws2812_blink_spi.py --count 30 --interval 0.2 --brightness 128
```

可选参数：
- `--bus/--device` 指定 spidev 设备（默认 0/0 -> /dev/spidev0.0）
- `--speed` SPI 速率（默认 2.4MHz）
- `--order` 颜色顺序（默认 GRB）

遇到颜色错乱或不稳定，可尝试：
- 降低 `--speed`，例如 2MHz 或 1.6MHz
- 加强电源滤波，靠近灯带并联 ≥1000µF 电容
- 使用 3.3V->5V 逻辑电平转换器
- 引脚选择：PWM0: BCM18，PWM1: BCM13；PCM: BCM21 等，具体以平台文档为准。
- 无法点亮或颜色错乱：尝试切换 `--strip-type` 为 RGB/GRB/BGR 等类型。
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

## 舵机 PWM 控制 (Jetson Orin NX)

新增脚本 `src/servo_pwm.py` 可在 Orin NX 的 40-pin 物理引脚 15 输出 50Hz 软件 PWM 用于驱动常见舵机。

注意事项:
- 舵机需要独立 5V 供电，且与 Orin NX 共地(GND 必须相连)。
- 软件 PWM 一般足够稳定；若需更稳可迁移到支持硬件 PWM 的引脚(如 32/33) 并启用硬件 PWM。
- 运行时可能需要 sudo 或将用户加入 gpio 组。

依赖安装(二选一):
- apt: sudo apt-get install python3-jetson-gpio
- pip: pip install Jetson.GPIO

基本用法:
- 固定角度(默认 90°，保持 1 秒):
   sudo -E python3 src/servo_pwm.py --pin 15 --angle 90 --hold 1
- 连续保持角度直到 Ctrl+C:
   sudo -E python3 src/servo_pwm.py --pin 15 --angle 45 --hold 0
- 扫动测试(0°~180° 来回):
   sudo -E python3 src/servo_pwm.py --pin 15 --sweep --step 3 --dwell 0.02
- 校准脉宽(常见范围 500us~2500us，少数舵机可能 600~2400us):
   sudo -E python3 src/servo_pwm.py --pin 15 --angle 90 --min-pulse 600 --max-pulse 2400

参数:
- --pin: 物理引脚号(BOARD)，默认 15
- --freq: PWM 频率，默认 50Hz
- --angle: 固定角度模式
- --sweep: 扫动模式
- --min-pulse/--max-pulse: 对应 0°/180° 的脉宽(微秒)
- --min-angle/--max-angle: 扫动或限制范围
- --step/--dwell: 扫动步进与步间延时
- --hold: 固定角度保持时间(<=0 表示持续)
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