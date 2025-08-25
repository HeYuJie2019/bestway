# Orin NX 内置CAN接口连接指南

## 🔌 硬件连接

### Orin NX CAN接口引脚位置
根据NVIDIA Orin NX开发套件的引脚定义，CAN接口通常在40针GPIO头上：

**标准CAN引脚分配：**
- **CAN0_RX** (接收): 通常在GPIO引脚上
- **CAN0_TX** (发送): 通常在GPIO引脚上

### 📍 具体连接方法

#### 方案1：使用CAN收发器模块（推荐）
```
CAR28F雷达    <->   CAN收发器   <->   Orin NX
CAN-H        <->   CANH        <->   
CAN-L        <->   CANL        <->   
VCC          <->   VCC         <->   3.3V/5V
GND          <->   GND         <->   GND
             <->   CTX         <->   CAN0_TX (GPIO引脚)
             <->   CRX         <->   CAN0_RX (GPIO引脚)
```

常用的CAN收发器模块：
- **TJA1050** (5V)
- **SN65HVD230** (3.3V)
- **MCP2515** + **TJA1050**

#### 方案2：检查是否有内置CAN收发器
某些Orin NX载板可能已经集成了CAN收发器，只需要连接：
```
CAR28F雷达直接连接到载板的CAN接口：
CAN-H -> 载板CAN-H
CAN-L -> 载板CAN-L
GND   -> 载板GND
```

## 🔍 如何确定您的载板配置

### 检查载板文档
1. 查看您的Orin NX载板(carrier board)型号
2. 检查载板的原理图或用户手册
3. 确认CAN接口的具体位置和类型

### 常见载板类型：
- **NVIDIA DevKit载板**: 通常CAN信号在40针GPIO头
- **第三方载板**: 可能有专用的CAN接口连接器
- **定制载板**: 需要查看具体的设计文档

## 🛠️ 连接验证步骤

### 1. 硬件连接检查
```bash
# 检查CAN接口状态
ip link show can0

# 检查CAN统计
cat /proc/net/can/stats
```

### 2. 设置CAN接口
```bash
# 关闭接口
sudo ip link set can0 down

# 配置波特率
sudo ip link set can0 type can bitrate 500000 restart-ms 100

# 启动接口
sudo ip link set can0 up

# 检查状态
ip -details link show can0
```

### 3. 测试连接
```bash
# 监听所有CAN消息
candump can0

# 发送测试消息
cansend can0 123#DEADBEEF
```

## ⚡ CAN总线要求

### 电气特性
- **差分信号**: CAN-H和CAN-L之间的电压差
- **终端电阻**: CAN总线两端各需要120Ω终端电阻
- **共地**: 所有设备必须共地

### 终端电阻配置
```
[Orin NX] ---- 120Ω ---- [CAN总线] ---- 120Ω ---- [CAR28F雷达]
```

如果总线上只有两个设备，每个设备端都需要120Ω电阻。

## 🔧 故障排除

### 1. 检查硬件连接
- [ ] CAN-H和CAN-L是否正确连接
- [ ] 电源和地线是否连接
- [ ] 终端电阻是否正确安装

### 2. 检查软件配置
- [ ] CAN接口是否启动
- [ ] 波特率是否匹配
- [ ] CAN驱动是否加载

### 3. 使用万用表测试
- CAN-H相对于GND：约2.5V(隐性)到3.5V(显性)
- CAN-L相对于GND：约2.5V(隐性)到1.5V(显性)
- CAN-H和CAN-L之间差分电压：0V(隐性)到2V(显性)

## 📚 相关命令参考

### CAN接口管理
```bash
# 查看所有网络接口
ip link show

# 重启CAN接口
sudo ip link set can0 down
sudo ip link set can0 up

# 查看CAN错误计数
cat /sys/class/net/can0/statistics/*
```

### CAN调试工具
```bash
# 安装can-utils
sudo apt install can-utils

# 实时监听
candump can0

# 发送数据
cansend can0 ID#DATA

# CAN总线负载分析
canbusload can0@500000
```

## 📋 下一步行动

1. **确认您的载板型号**
2. **查找载板的CAN引脚定义**
3. **准备合适的CAN收发器模块**（如果需要）
4. **按照连接图进行硬件连接**
5. **运行我们的测试程序验证连接**

请告诉我您的Orin NX载板型号，我可以提供更具体的连接指导！
