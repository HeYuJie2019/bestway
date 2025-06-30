#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
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
            1002, 1002, 1002, 1002, 1002, 1002, 282, 1722,
            282, 282, 1002, 1002, 300, 300, 300, 300
        ]
        self.default_channels = [
            1002, 1002, 1002, 1002, 1002, 1002, 282, 1722,
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
        self.boss = serial.Serial('/dev/ttyTHS1', 115200, timeout=1)
        self.BOSS = None  # 默认不处于 BOSS 模式
        self.boss.reset_input_buffer()
        self.boss.reset_output_buffer()

        # 订阅 /cmd_vel 话题
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # 订阅 /Odometry 话题
        # self.odom_subscription = self.create_subscription(
        #     Odometry,
        #     '/Odometry',
        #     self.odometry_callback,
        #     10
        # )

        # 定时器，用于检测是否停止发布
        self.last_cmd_time = self.get_clock().now()
        self.timer = self.create_timer(0.02, self.check_timeout)  # 每 0.02 秒触发一次

        self.serial_timer = self.create_timer(0.05, self.serial_send_and_read)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def serial_send_and_read(self):
        hex_data = '01'  # 你要发送的十六进制内容
        # self.boss.reset_output_buffer()  # 清空输入缓冲区
        # self.boss.reset_input_buffer()  # 清空输出缓冲区
        self.boss.write(bytes.fromhex(hex_data))
        # self.get_logger().info(f"串口发送: {hex_data}")
        # 读取串口数据
        if self.boss.in_waiting:
            # 读取所有数据，只取最后一个字节
            data = self.boss.read(self.boss.in_waiting)
            if data:
                last_byte = data[-1]
            self.get_logger().info(f"串口接收: {last_byte}")
            if last_byte == 1:
                self.BOSS = False
            elif last_byte == 2:
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
        self.send_sbus_frame()

    def odometry_callback(self, msg):
        # 根据/Odometry内容发布odom->base_link的TF
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

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
            # 读取CH341USB1的数据并转发
            if self.sbus_in.in_waiting >= 35:
                # 读取前清空输入缓冲区，只处理最新一帧
                while self.sbus_in.in_waiting >= 35:
                    data = self.sbus_in.read(35)
                if 'data' in locals():
                    channels, flag, xor = parse_sbus_frame(data)
                    if channels is not None:
                        # 只映射1,2,3,4,11,12通道
                        for idx in [0, 1, 2, 3, 10, 11]:
                            channels[idx] = map_channel(channels[idx])
                        # 处理第8通道（索引7）
                        if channels[7] == 1723:
                            channels[7] = 1722
                        # 其余通道保持不变
                        self.get_logger().info(f"SBUS 16通道数据: {channels}, flag: {flag:02X}, xor: {xor:02X}")
                        frame = create_sbus_frame(channels, flag)
                        self.serial_port.write(frame)
                    else:
                        self.get_logger().warn("SBUS帧校验失败或格式错误")
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