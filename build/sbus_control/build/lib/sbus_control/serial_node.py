#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
import serial
import time
from datetime import datetime

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        
        # 声明参数
        self.declare_parameter('port', '/dev/ttyTHS1')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 1.0)
        
        # 获取参数
        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        timeout = self.get_parameter('timeout').get_parameter_value().double_value
        
        # 初始化串口
        try:
            self.serial_port = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,    # 8位数据位
                parity=serial.PARITY_NONE,    # 无校验
                stopbits=serial.STOPBITS_ONE, # 1位停止位
                timeout=timeout,
                xonxoff=False,                # 禁用软件流控
                rtscts=False,                 # 禁用硬件流控
                dsrdtr=False                  # 禁用DTR/DSR
            )
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            self.get_logger().info(f"串口连接成功: {port}")
            self.get_logger().info(f"串口参数: {baudrate}bps, 8N1, 无流控")
        except Exception as e:
            self.get_logger().error(f"串口连接失败: {e}")
            self.serial_port = None
        
        # 存储当前坐标
        self.current_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        
        # 标志位：是否已接收到坐标数据
        self.has_received_odometry = False
        
        # 记录最后接收坐标的时间
        self.last_odometry_time = None
        
        # 超时时间（秒）
        self.odometry_timeout = 0.2
        
        # 订阅 /Odometry 话题
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/Odometry',
            self.odometry_callback,
            10
        )
        
        # 发布 /boss 话题
        self.boss_publisher = self.create_publisher(Int32, '/boss', 10)
        
        # 定时器，用于发送坐标和读取数据
        self.timer = self.create_timer(0.1, self.serial_communication)  # 改为100ms，降低发送频率
        
        # 添加发送锁，防止数据冲突
        self.sending_lock = False
        
        self.get_logger().info("串口节点已启动")
    
    def odometry_callback(self, msg):
        """
        接收里程计数据，更新当前坐标
        """
        self.current_position['x'] = msg.pose.pose.position.x
        self.current_position['y'] = msg.pose.pose.position.y
        self.current_position['z'] = msg.pose.pose.position.z
        
        # 更新最后接收时间
        self.last_odometry_time = self.get_clock().now()
        
        # 标记已接收到坐标数据
        if not self.has_received_odometry:
            self.has_received_odometry = True
            self.get_logger().info("首次接收到坐标数据，开始串口通信")
        
        self.get_logger().debug(f"接收坐标: X={self.current_position['x']:.2f}, "
                               f"Y={self.current_position['y']:.2f}, "
                               f"Z={self.current_position['z']:.2f}")
    
    def send_coordinates(self):
        """
        发送坐标信息到串口
        格式: 帧头(2字节) + X坐标(4字节) + Y坐标(4字节) + Z坐标(4字节) + 帧尾(2字节)
        总长度: 16字节，坐标采用4字节有符号整数，小端序编码
        """
        if not self.serial_port or not self.serial_port.is_open:
            return False
        
        # 防止并发发送
        if self.sending_lock:
            return False
            
        try:
            self.sending_lock = True
            
            x = self.current_position['x']
            y = self.current_position['y']
            z = self.current_position['z']
            
            # 将浮点数转换为定点数（乘以100保留两位小数）
            x_int = int(x * 100)
            y_int = int(y * 100) 
            z_int = int(z * 100)
            
            # 构建数据包
            data = bytearray()
            
            # 帧头: 0xAA 0x55
            data.extend([0xAA, 0x55])
            
            # X坐标（4字节有符号整数，小端序）
            data.extend(x_int.to_bytes(4, byteorder='little', signed=True))
            
            # Y坐标（4字节有符号整数，小端序）
            data.extend(y_int.to_bytes(4, byteorder='little', signed=True))
            
            # Z坐标（4字节有符号整数，小端序）
            data.extend(z_int.to_bytes(4, byteorder='little', signed=True))
            
            # 帧尾: 0x0D 0x0A
            data.extend([0x0D, 0x0A])
            
            # 发送数据
            self.serial_port.write(data)
            self.serial_port.flush()
            
            # 生成16进制字符串用于日志
            hex_str = ' '.join([f'{b:02X}' for b in data])
            self.get_logger().info(f"发送坐标: {hex_str}")
            self.get_logger().info(f"坐标值: X={x:.2f}, Y={y:.2f}, Z={z:.2f}")
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"发送坐标失败: {e}")
            return False
        finally:
            self.sending_lock = False
    
    def read_serial_data(self):
        """
        读取串口数据并解析BOSS状态
        """
        if not self.serial_port or not self.serial_port.is_open:
            return
        
        try:
            if self.serial_port.in_waiting > 0:
                # 读取所有可用数据
                data = self.serial_port.read(self.serial_port.in_waiting)
                
                self.get_logger().debug(f"接收原始数据: {data.hex()}")
                
                # 解析BOSS状态数据
                if data:
                    frames = self.parse_frames(data)
                    for frame in frames:
                        self.process_boss_frame(frame)
                
        except Exception as e:
            self.get_logger().error(f"读取串口数据失败: {e}")
    
    def parse_frames(self, data):
        """
        解析数据帧：帧头(0xAA55) + 数据 + 帧尾(0x0D0A)
        """
        frames = []
        buffer = data
        
        while len(buffer) >= 4:  # 至少需要帧头+帧尾的长度
            # 查找帧头 0xAA55
            header_pos = -1
            for i in range(len(buffer) - 1):
                if buffer[i] == 0xAA and buffer[i + 1] == 0x55:
                    header_pos = i
                    break
            
            if header_pos == -1:
                break  # 没有找到帧头
            
            # 查找帧尾 0x0D0A
            tail_pos = -1
            for i in range(header_pos + 2, len(buffer) - 1):
                if buffer[i] == 0x0D and buffer[i + 1] == 0x0A:
                    tail_pos = i
                    break
            
            if tail_pos == -1:
                break  # 没有找到帧尾
            
            # 提取完整帧（包括帧头和帧尾）
            frame_data = buffer[header_pos:tail_pos + 2]
            frames.append(frame_data)
            
            # 移除已处理的数据
            buffer = buffer[tail_pos + 2:]
        
        return frames
    
    def process_boss_frame(self, frame):
        """
        处理BOSS状态数据帧
        """
        if len(frame) < 4:
            return
        
        try:
            # 提取数据部分（去掉帧头和帧尾）
            data_part = frame[2:-2]
            
            # 生成16进制字符串用于日志
            hex_str = ' '.join([f'{b:02X}' for b in frame])
            self.get_logger().info(f"接收到帧: {hex_str}")
            
            # 如果数据部分只有1个字节，认为是BOSS状态
            if len(data_part) == 1:
                status_byte = data_part[0]
                
                # 创建并发布BOSS状态消息
                boss_msg = Int32()
                
                if status_byte == 1:
                    boss_msg.data = 0  # 非BOSS模式
                    self.get_logger().info("BOSS状态: 非BOSS模式")
                elif status_byte == 2:
                    boss_msg.data = 1  # BOSS模式
                    self.get_logger().info("BOSS状态: BOSS模式")
                else:
                    boss_msg.data = -1  # 未知状态
                    self.get_logger().info(f"BOSS状态: 未知 ({status_byte})")
                
                # 发布状态
                self.boss_publisher.publish(boss_msg)
            else:
                self.get_logger().info(f"接收到其他数据: {data_part.hex()}")
                
        except Exception as e:
            self.get_logger().error(f"处理数据帧失败: {e}")
    
    def is_odometry_timeout(self):
        """
        检查坐标数据是否超时
        """
        if not self.has_received_odometry or self.last_odometry_time is None:
            return True
        
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_odometry_time).nanoseconds / 1e9
        
        return time_diff > self.odometry_timeout

    def serial_communication(self):
        """
        定时执行的串口通信函数
        先发送坐标，再读取数据
        只有在接收到坐标数据且未超时的情况下才发送
        """
        # 如果正在发送数据，跳过这次循环
        if self.sending_lock:
            return
            
        # 检查是否接收到坐标数据且未超时
        if not self.has_received_odometry:
            self.get_logger().info("等待接收坐标数据...")
            # 仍然读取数据以保持通信
            self.read_serial_data()
            return
        
        # 检查坐标数据是否超时
        if self.is_odometry_timeout():
            self.get_logger().info(f"坐标数据超时 ({self.odometry_timeout}秒)，停止发送坐标")
            # 仍然读取数据以保持通信
            self.read_serial_data()
            return
        
        # 发送坐标
        if self.send_coordinates():
            # 发送成功后等待更长时间，确保数据传输完成
            time.sleep(0.05)
        
        # 读取数据
        self.read_serial_data()
    
    def destroy_node(self):
        """
        节点销毁时关闭串口
        """
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info("串口已关闭")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SerialNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
