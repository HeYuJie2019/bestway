# USB设备固定名称配置说明

## 概述
为了解决USB设备每次插入时名称可能改变的问题，我们使用udev规则为CH341 USB串口设备创建了固定的符号链接：
- `/dev/ttyCH341USB0` → `/dev/ttyUSB_SBUS`
- `/dev/ttyCH341USB1` → `/dev/ttySBUS_USB`

## 配置文件
- **udev规则文件**: `/etc/udev/rules.d/99-usb-serial-fixed.rules`
- **源码备份**: `/home/bestway/bestway_ws/99-usb-serial-fixed.rules`

## 设备信息
### 设备1 (原为 SBUS 控制)
- **原设备名**: `/dev/ttyCH341USB0`
- **固定名称**: `/dev/ttyUSB_SBUS`
- **厂商ID**: 1a86 (QinHeng Electronics)
- **产品ID**: 7523 (CH340 serial converter)
- **设备路径**: 2.4

### 设备2 (原为 SBUS 输入)
- **原设备名**: `/dev/ttyCH341USB1`
- **固定名称**: `/dev/ttySBUS_USB`
- **厂商ID**: 1a86 (QinHeng Electronics)
- **产品ID**: 7523 (CH340 serial converter)
- **设备路径**: 2.1

## 已更新的代码文件
1. `/home/bestway/bestway_ws/src/control.py`
2. `/home/bestway/bestway_ws/src/sbus_control/sbus_control/sbus_control_node.py`
3. `/home/bestway/bestway_ws/src/serial_reader_rader.py`

## 验证方法
```bash
# 检查符号链接是否存在
ls -la /dev/ttyUSB_SBUS /dev/ttySBUS_USB

# 验证链接指向正确的设备
ls -la /dev/ttyUSB_SBUS /dev/ttyCH341USB0
ls -la /dev/ttySBUS_USB /dev/ttyCH341USB1

# 测试串口是否可用
echo "测试设备1" > /dev/ttyUSB_SBUS
echo "测试设备2" > /dev/ttySBUS_USB
```

## 故障排除

### 如果设备名称仍然不固定
1. 检查udev规则是否正确加载：
   ```bash
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```

2. 重新测试规则：
   ```bash
   udevadm test $(udevadm info -q path -n /dev/ttyCH341USB0)
   ```

3. 检查设备属性是否匹配：
   ```bash
   udevadm info -a -n /dev/ttyCH341USB0 | grep -E "(idVendor|idProduct|devpath)"
   ```

### 如果需要为其他USB设备创建固定名称
1. 获取设备信息：
   ```bash
   udevadm info -a -n /dev/设备名称
   ```

2. 在 `/etc/udev/rules.d/99-usb-serial-fixed.rules` 中添加新规则：
   ```
   SUBSYSTEM=="tty", ATTRS{idVendor}=="厂商ID", ATTRS{idProduct}=="产品ID", ATTRS{devpath}=="设备路径", SYMLINK+="自定义名称"
   ```

3. 重新加载规则：
   ```bash
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```

## 注意事项
1. 如果USB设备插入到不同的USB端口，可能需要更新规则中的 `devpath` 属性
2. 如果有多个相同型号的设备，需要使用不同的标识符（如设备路径）来区分
3. 每次修改udev规则后都需要重新加载
4. 建议定期备份udev规则文件

## 重新编译项目
更新代码后，需要重新编译ROS2包：
```bash
cd /home/bestway/bestway_ws
colcon build --packages-select sbus_control
source install/setup.bash
```
