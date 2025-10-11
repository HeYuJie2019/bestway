#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
import serial
import time
from datetime import datetime
import subprocess
import psutil
import os
import signal

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        
        # 声明参数
        # self.declare_parameter('port', '/dev/ttyTHS1')
        # self.declare_parameter('port', '/dev/ttyCH341USB1')
        self.declare_parameter('port', '/dev/ttyCH9344USB5')
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
        self.odometry_timeout = 0.3
        
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
        
        # 添加03 00命令的时间锁定机制（6秒内不重复处理）
        self.last_03_00_time = None
        self.cmd_03_00_lock_duration = 6.0  # 6秒锁定时间
        
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
        解析数据帧：帧头(0xAABB) + 数据(2字节) + 帧尾(0xCCDD)
        总长度：6字节
        """
        frames = []
        buffer = data
        
        while len(buffer) >= 6:  # 完整帧长度为6字节
            # 查找帧头 0xAABB
            header_pos = -1
            for i in range(len(buffer) - 5):  # 确保有足够空间容纳完整帧
                if buffer[i] == 0xAA and buffer[i + 1] == 0xBB:
                    header_pos = i
                    break
            
            if header_pos == -1:
                break  # 没有找到帧头
            
            # 检查是否有完整的6字节帧
            if header_pos + 6 > len(buffer):
                break  # 数据不够一个完整帧
            
            # 检查帧尾是否正确 0xCCDD
            if (buffer[header_pos + 4] == 0xCC and 
                buffer[header_pos + 5] == 0xDD):
                # 提取完整帧（6字节）
                frame_data = buffer[header_pos:header_pos + 6]
                frames.append(frame_data)
                
                # 移除已处理的数据
                buffer = buffer[header_pos + 6:]
            else:
                # 帧尾不正确，跳过当前帧头，继续查找
                buffer = buffer[header_pos + 1:]
        
        return frames
    
    def is_process_running(self, cmd_keyword):
        """
        检查包含指定关键字的进程是否正在运行
        """
        if cmd_keyword == 'mapping.launch.py':
            # 对于建图节点，检查相关关键词但排除livox驱动
            search_keywords = ['mapping.launch.py', 'fast_lio']
            for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                try:
                    cmdline_str = ' '.join(proc.info['cmdline']) if proc.info['cmdline'] else ''
                    for keyword in search_keywords:
                        if keyword in cmdline_str and 'livox' not in cmdline_str.lower():
                            return proc.info['pid']
                except Exception:
                    continue
        elif cmd_keyword == 'search_fire_node':
            # 对于search_fire_node，专门检查
            search_keywords = ['search_fire_node', 'auto_drive']
            for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                try:
                    cmdline_str = ' '.join(proc.info['cmdline']) if proc.info['cmdline'] else ''
                    for keyword in search_keywords:
                        if keyword in cmdline_str and 'search_fire' in cmdline_str:
                            return proc.info['pid']
                except Exception:
                    continue
        else:
            # 对于其他进程，使用原有逻辑
            for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                try:
                    if any(cmd_keyword in str(x) for x in proc.info['cmdline']):
                        return proc.info['pid']
                except Exception:
                    continue
        return None

    def start_mapping_node(self):
        """启动建图节点"""
        self.get_logger().info("启动建图节点: ros2 launch fast_lio mapping.launch.py")
        subprocess.Popen(['ros2', 'launch', 'fast_lio', 'mapping.launch.py'])

    def stop_mapping_node(self):
        """停止建图节点（杀死所有相关进程）"""
        killed_pids = []
        
        # 只搜索建图相关的进程，不包含livox驱动
        search_keywords = ['mapping.launch.py', 'fast_lio']
        
        # 首先收集所有需要杀死的进程ID
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                cmdline_str = ' '.join(proc.info['cmdline']) if proc.info['cmdline'] else ''
                # 检查是否包含建图相关关键词，但排除livox驱动
                for keyword in search_keywords:
                    if keyword in cmdline_str and 'livox' not in cmdline_str.lower():
                        if proc.info['pid'] not in killed_pids:
                            killed_pids.append(proc.info['pid'])
                            self.get_logger().info(f"找到建图相关进程: PID={proc.info['pid']}, CMD={cmdline_str}")
                        break
            except Exception:
                continue
        
        # 杀死所有找到的进程
        for pid in killed_pids:
            try:
                self.get_logger().info(f"强制停止进程，PID={pid}")
                process = psutil.Process(pid)
                
                # 首先尝试杀死子进程
                try:
                    children = process.children(recursive=True)
                    for child in children:
                        self.get_logger().info(f"杀死子进程，PID={child.pid}")
                        child.kill()
                        child.wait(timeout=2)
                except Exception as e:
                    self.get_logger().warn(f"杀死子进程失败: {e}")
                
                # 然后杀死主进程
                process.kill()
                process.wait(timeout=5)
                self.get_logger().info(f"进程PID={pid}已停止")
                
            except psutil.TimeoutExpired:
                self.get_logger().warn(f"进程PID={pid}停止超时，尝试强制杀死")
                try:
                    # 最后的强制手段
                    import os
                    import signal
                    os.kill(pid, signal.SIGKILL)
                except Exception as e:
                    self.get_logger().error(f"强制杀死进程PID={pid}失败: {e}")
            except psutil.NoSuchProcess:
                self.get_logger().info(f"进程PID={pid}已不存在")
            except Exception as e:
                self.get_logger().error(f"停止进程PID={pid}失败: {e}")
        
        if not killed_pids:
            self.get_logger().info("建图节点未运行，无需停止")

    def restart_mapping_node(self):
        """重启建图节点"""
        self.get_logger().info("开始重启建图节点...")
        self.stop_mapping_node()
        # 等待更长时间确保所有进程完全退出
        self.get_logger().info("等待进程完全退出...")
        time.sleep(5)
        # 再次检查是否还有残留进程
        remaining_pids = []
        search_keywords = ['mapping.launch.py', 'fast_lio', 'mapping']
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                cmdline_str = ' '.join(proc.info['cmdline']) if proc.info['cmdline'] else ''
                for keyword in search_keywords:
                    if keyword in cmdline_str:
                        remaining_pids.append(proc.info['pid'])
                        break
            except Exception:
                continue
        
        if remaining_pids:
            self.get_logger().warn(f"仍有残留进程: {remaining_pids}")
            for pid in remaining_pids:
                try:
                    os.kill(pid, signal.SIGKILL)
                    self.get_logger().info(f"强制杀死残留进程: {pid}")
                except Exception as e:
                    self.get_logger().error(f"杀死残留进程{pid}失败: {e}")
            time.sleep(2)
        
        self.get_logger().info("启动新的建图节点...")
        self.start_mapping_node()

    def start_search_fire_node(self):
        self.get_logger().info("启动search_fire_node: ros2 run auto_drive search_fire_node")
        subprocess.Popen(['ros2', 'run', 'auto_drive', 'search_fire_node'])

    def stop_search_fire_node(self):
        """停止search_fire_node节点（彻底杀死所有相关进程）"""
        killed_pids = []
        
        # 搜索search_fire_node相关的进程
        search_keywords = ['search_fire_node', 'auto_drive']
        
        # 首先收集所有需要杀死的进程ID
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                cmdline_str = ' '.join(proc.info['cmdline']) if proc.info['cmdline'] else ''
                # 检查是否包含search_fire_node相关关键词
                for keyword in search_keywords:
                    if keyword in cmdline_str and 'search_fire' in cmdline_str:
                        if proc.info['pid'] not in killed_pids:
                            killed_pids.append(proc.info['pid'])
                            self.get_logger().info(f"找到search_fire_node相关进程: PID={proc.info['pid']}, CMD={cmdline_str}")
                            break
            except Exception:
                continue
        
        # 杀死所有找到的进程
        for pid in killed_pids:
            try:
                self.get_logger().info(f"强制停止search_fire_node节点，PID={pid}")
                process = psutil.Process(pid)
                
                # 首先尝试杀死所有子进程
                try:
                    children = process.children(recursive=True)
                    for child in children:
                        self.get_logger().info(f"杀死子进程，PID={child.pid}")
                        child.kill()
                        child.wait(timeout=2)
                except Exception as e:
                    self.get_logger().warn(f"杀死子进程失败: {e}")
                
                # 然后杀死主进程
                process.kill()
                process.wait(timeout=5)
                self.get_logger().info(f"search_fire_node进程PID={pid}已停止")
                
            except psutil.TimeoutExpired:
                self.get_logger().warn(f"search_fire_node进程PID={pid}停止超时，尝试强制杀死")
                try:
                    # 最后的强制手段
                    import os
                    import signal
                    os.kill(pid, signal.SIGKILL)
                except Exception as e:
                    self.get_logger().error(f"强制杀死search_fire_node进程PID={pid}失败: {e}")
            except psutil.NoSuchProcess:
                self.get_logger().info(f"search_fire_node进程PID={pid}已不存在")
            except Exception as e:
                self.get_logger().error(f"停止search_fire_node进程PID={pid}失败: {e}")
        
        if not killed_pids:
            self.get_logger().info("search_fire_node未运行，无需停止")

    def restart_search_fire_node(self):
        """重启search_fire_node节点"""
        self.get_logger().info("开始重启search_fire_node节点...")
        self.stop_search_fire_node()
        # 等待一段时间确保所有进程完全退出
        self.get_logger().info("等待search_fire_node进程完全退出...")
        time.sleep(2)
        # 再次检查是否还有残留进程
        remaining_pids = []
        search_keywords = ['search_fire_node', 'auto_drive']
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                cmdline_str = ' '.join(proc.info['cmdline']) if proc.info['cmdline'] else ''
                for keyword in search_keywords:
                    if keyword in cmdline_str and 'search_fire' in cmdline_str:
                        remaining_pids.append(proc.info['pid'])
                        break
            except Exception:
                continue
        
        if remaining_pids:
            self.get_logger().warn(f"仍有search_fire_node残留进程: {remaining_pids}")
            for pid in remaining_pids:
                try:
                    import os
                    import signal
                    os.kill(pid, signal.SIGKILL)
                    self.get_logger().info(f"强制杀死search_fire_node残留进程: {pid}")
                except Exception as e:
                    self.get_logger().error(f"杀死search_fire_node残留进程{pid}失败: {e}")
        
        self.get_logger().info("启动search_fire_node节点...")
        self.start_search_fire_node()

    def process_boss_frame(self, frame):
        """
        处理BOSS状态数据帧
        协议格式：帧头(AA BB) + 数据(2字节) + 帧尾(CC DD)
        数据含义：
        - 01 00 = 启动search_fire_node（如果未运行）
        - 02 00 = 停止search_fire_node
        - 03 00 = 启动/重启建图节点（6秒内重复命令忽略）
        """
        if len(frame) != 6:
            self.get_logger().warn(f"帧长度错误，期望6字节，实际{len(frame)}字节")
            return
        try:
            hex_str = ' '.join([f'{b:02X}' for b in frame])
            self.get_logger().info(f"接收到帧: {hex_str}")
            if frame[0] != 0xAA or frame[1] != 0xBB:
                self.get_logger().warn(f"帧头错误: {frame[0]:02X} {frame[1]:02X}")
                return
            if frame[4] != 0xCC or frame[5] != 0xDD:
                self.get_logger().warn(f"帧尾错误: {frame[4]:02X} {frame[5]:02X}")
                return
            data_byte1 = frame[2]
            data_byte2 = frame[3]
            self.get_logger().info(f"数据部分: {data_byte1:02X} {data_byte2:02X}")
            boss_msg = Int32()
            # 新协议逻辑
            if (data_byte1, data_byte2) == (0x03, 0x00):
                boss_msg.data = 1
                self.get_logger().info("收到03 00命令")
                # 检查是否在6秒锁定期内
                current_time = self.get_clock().now()
                if self.last_03_00_time is not None:
                    time_diff = (current_time - self.last_03_00_time).nanoseconds / 1e9
                    if time_diff < self.cmd_03_00_lock_duration:
                        self.get_logger().info(f"接收到03 00，但在锁定期内({time_diff:.1f}s < {self.cmd_03_00_lock_duration}s)，忽略处理")
                        # 仍然发布BOSS状态，但不执行建图节点操作
                        self.boss_publisher.publish(boss_msg)
                        return
                
                # 更新最后处理03 00的时间
                self.last_03_00_time = current_time
                self.get_logger().info("处理03 00命令，启动/重启建图节点")
                
                # 建图节点管理
                if self.is_process_running('mapping.launch.py'):
                    self.get_logger().info("建图节点已运行，重启...")
                    self.restart_mapping_node()
                else:
                    self.get_logger().info("建图节点未运行，启动...")
                    self.start_mapping_node()
            elif (data_byte1, data_byte2) == (0x01, 0x00):
                boss_msg.data = 0
                self.get_logger().info("收到01 00命令")
                # search_fire_node管理：只在未运行时启动
                if self.is_process_running('search_fire_node'):
                    self.get_logger().info("search_fire_node已运行，忽略01 00命令")
                else:
                    self.get_logger().info("search_fire_node未运行，启动...")
                    self.start_search_fire_node()
            elif (data_byte1, data_byte2) == (0x02, 0x00):
                boss_msg.data = 1
                self.get_logger().info("收到02 00命令，停止search_fire_node")
                # 停止search_fire_node节点
                if self.is_process_running('search_fire_node'):
                    self.get_logger().info("search_fire_node已运行，停止...")
                    self.stop_search_fire_node()
                else:
                    self.get_logger().info("search_fire_node未运行，无需停止")
            else:
                boss_msg.data = -1
                self.get_logger().warn(f"未知状态 ({data_byte1:02X} {data_byte2:02X})")
            self.boss_publisher.publish(boss_msg)
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
