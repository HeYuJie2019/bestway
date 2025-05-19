#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

def calculate_xor(data):
    xor = 0
    for byte in data:
        xor ^= byte
    return xor

def create_sbus_frame(channels, flag=0x00):
    if len(channels) != 16:
        raise ValueError("需要 16 个通道值")
    channels = [min(max(ch, 0), 2047) for ch in channels]
    frame = [0x0F]
    for ch in channels:
        frame.append((ch >> 8) & 0xFF)
        frame.append(ch & 0xFF)
    frame.append(flag)
    xor = calculate_xor(frame[1:])
    frame.append(xor)
    return bytes(frame)

class SbusControlNode(Node):
    def __init__(self):
        super().__init__('sbus_control_node')

        # 获取参数
        self.declare_parameter('port', '/dev/ttyCH341USB0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        # 初始化串口
        self.channels = [
            1002, 1002, 1002, 1002, 1002, 1002, 282, 282,
            282, 282, 1002, 1002, 1002, 1002, 1002, 1002
        ]
        self.default_channels = [
            1002, 1002, 1002, 1002, 1002, 1002, 282, 282,
            282, 282, 1002, 1002, 1002, 1002, 1002, 1002
        ]
        self.serial_port = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_EVEN,
            stopbits=serial.STOPBITS_TWO
        )

        self.boss = serial.Serial('/dev/ttyTHS1', 115200, timeout=1)
        self.BOSS = None

        # 订阅 /cmd_vel 话题
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # 定时器，用于检测是否停止发布
        self.last_cmd_time = self.get_clock().now()
        self.timer = self.create_timer(0.02, self.check_timeout)  # 每 0.02 秒触发一次

        self.serial_timer = self.create_timer(0.05, self.serial_send_and_read)

    def serial_send_and_read(self):
        hex_data = '01'  # 你要发送的十六进制内容
        self.boss.write(bytes.fromhex(hex_data))
        self.get_logger().info(f"串口发送: {hex_data}")
        # 读取串口数据
        if self.boss.in_waiting:
            data = self.boss.read(self.boss.in_waiting).hex()
            if data == '01':
                self.BOSS = False
            elif data == '02':
                self.BOSS = True

    def cmd_vel_callback(self, msg):
        """
        当接收到 /cmd_vel 消息时，更新 channels 数据
        """
        self.last_cmd_time = self.get_clock().now()  # 更新最后接收时间

        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # 将线速度映射到 channels[2] (前进/后退)
        if linear_x > 0:
            self.channels[2] = int(1002 - 100 * linear_x)
        elif linear_x < 0:
            self.channels[2] = int(1002 - 100 * linear_x)
        else:
            self.channels[2] = 1002

        # 将角速度映射到 channels[0] (左转/右转)
        if angular_z > 0:
            self.channels[0] = int(1002 - 100 * angular_z)
        elif angular_z < 0:
            self.channels[0] = int(1002 - 100 * angular_z)
        else:
            self.channels[0] = 1002

        # 发送 SBUS 数据帧
        if self.BOSS is not True:
            self.send_sbus_frame()

    def check_timeout(self):
        """
        检查是否超时未接收到 /cmd_vel 消息
        如果超时，则发送默认的 channels 数据
        """
        now = self.get_clock().now()
        # 检查是否超时
        if (now - self.last_cmd_time).nanoseconds > 500_000_000:  # 超过 0.5 秒未接收到消息
            self.channels = self.default_channels.copy()
            if self.BOSS is not True:
                self.send_sbus_frame()

    def send_sbus_frame(self):
        """
        发送 SBUS 数据帧
        """
        frame = create_sbus_frame(self.channels)
        self.serial_port.write(frame)
        self.get_logger().info(f"发送数据帧: {frame.hex().upper()}")

def main(args=None):
    rclpy.init(args=args)
    node = SbusControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()