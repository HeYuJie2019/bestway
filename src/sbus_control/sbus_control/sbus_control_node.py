#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import serial
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

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

def parse_sbus_frame(data):
    """
    解析35字节SBUS数据帧，返回16通道、flag、校验
    """
    if len(data) != 35 or data[0] != 0x0F:
        return None, None, None

    # 16通道，每通道2字节，高字节在前
    channels = []
    for i in range(16):
        high = data[1 + i*2]
        low = data[2 + i*2]
        value = (high << 8) | low
        channels.append(value)

    flag = data[33]
    xor = data[34]
    # 校验
    calc_xor = 0
    for b in data[1:34]:
        calc_xor ^= b
    if xor != calc_xor:
        return None, None, None

    return channels, flag, xor

def map_channel(val, old_mid=1020, new_mid=1002):
    # 线性平移映射，保持比例
    return val - old_mid + new_mid

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
            282, 282, 1002, 1002, 300, 300, 300, 300
        ]
        self.default_channels = [
            1002, 1002, 1002, 1002, 1002, 1002, 282, 282,
            282, 282, 1002, 1002, 300, 300, 300, 300
        ]
        self.serial_port = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_EVEN,
            stopbits=serial.STOPBITS_TWO
        )
        # self.sbus_in = serial.Serial('/dev/ttyCH341USB1', 115200, timeout=0.01)
        self.BOSS = None  # 默认不处于 BOSS 模式

        # 订阅 /cmd_vel 话题
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # 订阅 /boss 话题
        self.boss_subscription = self.create_subscription(
            Int32,
            '/boss',
            self.boss_callback,
            10
        )

        # 定时器，用于检测是否停止发布
        self.last_cmd_time = self.get_clock().now()
        self.timer = self.create_timer(0.02, self.check_timeout)  # 每 0.02 秒触发一次

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def boss_callback(self, msg):
        """
        接收BOSS状态消息
        """
        if msg.data == 0:
            self.BOSS = False
            self.get_logger().info("接收BOSS状态: 非BOSS模式")
        elif msg.data == 1:
            self.BOSS = True
            self.get_logger().info("接收BOSS状态: BOSS模式")
        else:
            self.BOSS = None
            self.get_logger().info(f"接收BOSS状态: 未知状态 ({msg.data})")

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
            self.send_sbus_frame()

    def send_sbus_frame(self):
        """
        发送 SBUS 数据帧
        """
        if self.BOSS is True:
            # 读取CH341USB2的数据并直接转发
            if self.sbus_in.in_waiting >= 35:
                # 读取前清空输入缓冲区，只处理最新一帧
                while self.sbus_in.in_waiting >= 35:
                    data = self.sbus_in.read(35)
                if 'data' in locals():
                    # 直接转发原始数据，不做任何解析和映射
                    self.serial_port.write(data)
                    self.get_logger().info(f"直接转发SBUS数据: {data.hex().upper()}")
        else:
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