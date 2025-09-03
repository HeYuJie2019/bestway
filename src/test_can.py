#!/usr/bin/env python3
"""
CAN总线测试程序 - 用于Orin NX通过USB连接CAN盒子读取CAR28F传感器数据
根据CAR28F应用手册的准确消息定义配置
"""

import can
import time
import sys
import struct
import signal
import math

class CAR28FSensorReader:
    def __init__(self, interface='can0', bitrate=500000, radar_id=0):
        """
        初始化CAR28F传感器读取器
        
        Args:
            interface: CAN接口名称 (默认: can0)
            bitrate: CAN总线比特率 (CAR28F通常使用500kbps)
            radar_id: 雷达ID (默认: 0, 用于计算具体的CAN消息ID)
        """
        self.interface = interface
        self.bitrate = bitrate
        self.radar_id = radar_id
        self.bus = None
        self.running = False
        
        # CAR28F雷达帧消息定义
        # ID计算公式：每个雷达消息ID = 雷达ID * 0x10 + 基础消息ID
        self.base_message_ids = {
            'RadarConfiguration': 0x200,   # 雷达配置 (外部输入)
            'RadarFeedback': 0x400,        # 雷达回复 (CAR28F输出)
            'RadarStatus': 0x60A,          # 雷达状态输出 (CAR28F输出)
            'TargetStatus': 0x70B,         # 雷达目标状态 (CAR28F输出)
            'TargetInformation': 0x70C     # 雷达目标信息 (CAR28F输出)
        }
        
        # 计算实际的CAN消息ID
        self.message_ids = {}
        for msg_name, base_id in self.base_message_ids.items():
            self.message_ids[msg_name] = self.radar_id * 0x10 + base_id
        
        print(f"CAR28F雷达ID: {self.radar_id}")
        print("计算后的CAN消息ID:")
        for msg_name, msg_id in self.message_ids.items():
            print(f"  {msg_name}: 0x{msg_id:03X}")
        
        # 设置信号处理器用于优雅退出
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
    
    def extract_bits_from_int(self, data_int, start_bit, length):
        """
        从整数中提取指定位域的值
        
        Args:
            data_int: 源整数数据
            start_bit: 起始位位置
            length: 位域长度
            
        Returns:
            提取的位域值
        """
        # 创建掩码
        mask = (1 << length) - 1
        # 右移到目标位置并应用掩码
        return (data_int >> start_bit) & mask
    
    def signal_handler(self, signum, frame):
        """信号处理器，用于优雅退出"""
        print(f"\n收到信号 {signum}, 正在关闭CAN连接...")
        self.stop()
        sys.exit(0)
    
    def setup_can_interface(self):
        """设置CAN接口"""
        try:
            import subprocess
            
            # 关闭CAN接口
            subprocess.run(['sudo', 'ip', 'link', 'set', self.interface, 'down'], 
                          capture_output=True, check=False)
            
            # 设置CAN接口比特率 (CAR28F通常使用250kbps)
            subprocess.run(['sudo', 'ip', 'link', 'set', self.interface, 'type', 'can', 
                          'bitrate', str(self.bitrate)], 
                          capture_output=True, check=True)
            
            # 启动CAN接口
            subprocess.run(['sudo', 'ip', 'link', 'set', self.interface, 'up'], 
                          capture_output=True, check=True)
            
            print(f"CAN接口 {self.interface} 设置成功")
            print(f"比特率: {self.bitrate} bps (CAR28F标准)")
            return True
            
        except subprocess.CalledProcessError as e:
            print(f"设置CAN接口失败: {e}")
            return False
        except Exception as e:
            print(f"设置CAN接口时发生错误: {e}")
            return False
    
    def connect(self):
        """连接到CAN总线"""
        try:
            # 尝试连接CAN总线
            self.bus = can.interface.Bus(channel=self.interface, 
                                       bustype='socketcan',
                                       receive_own_messages=False)
            print(f"成功连接到CAN总线: {self.interface}")
            print(f"监听消息ID范围:")
            for msg_name, msg_id in self.message_ids.items():
                print(f"  {msg_name}: 0x{msg_id:03X}")
            return True
            
        except Exception as e:
            print(f"连接CAN总线失败: {e}")
            print("请检查:")
            print("1. CAN接口是否存在 (使用 'ip link show' 查看)")
            print("2. 是否有足够的权限")
            print("3. CAN盒子是否正确连接")
            print("4. CAR28F传感器是否正确供电和连接")
            return False
    
    def parse_car28f_data(self, message_id, data):
        """
        解析CAR28F传感器数据
        根据不同的消息ID解析相应的数据格式
        """
        try:
            # 确定消息类型
            message_type = None
            for msg_name, msg_id in self.message_ids.items():
                if message_id == msg_id:
                    message_type = msg_name
                    break
            
            if not message_type:
                return {
                    'error': f'Unknown message ID: 0x{message_id:03X}',
                    'raw_data': data.hex(),
                    'bytes': list(data)
                }
            
            result = {
                'message_type': message_type,
                'message_id': f'0x{message_id:03X}',
                'raw_data': data.hex(),
                'timestamp': time.time()
            }
            
            if message_type == 'RadarFeedback':
                # 雷达回复消息解析
                if len(data) >= 8:
                    result.update({
                        'feedback_status': data[0],
                        'error_code': data[1],
                        'version_major': data[2],
                        'version_minor': data[3],
                        'radar_id_feedback': data[4],
                        'configuration_ack': bool(data[5] & 0x01)
                    })
                    
            elif message_type == 'RadarStatus':
                # 雷达状态输出解析
                if len(data) >= 8:
                    status_byte = data[0]
                    result.update({
                        'radar_power_on': bool(status_byte & 0x01),
                        'radar_ready': bool(status_byte & 0x02),
                        'radar_error': bool(status_byte & 0x04),
                        'temperature_alarm': bool(status_byte & 0x08),
                        'voltage_alarm': bool(status_byte & 0x10),
                        'current_temperature': struct.unpack('<h', data[1:3])[0] / 10.0,  # 温度，单位0.1℃
                        'supply_voltage': struct.unpack('<H', data[3:5])[0] / 100.0,     # 电压，单位0.01V
                        'radar_frequency': struct.unpack('<H', data[5:7])[0],            # 工作频率
                        'measurement_cycle': data[7]  # 测量周期
                    })
                    
            elif message_type == 'TargetStatus':
                # 雷达目标状态解析
                if len(data) >= 8:
                    result.update({
                        'target_count': data[0],                    # 检测到的目标数量
                        'max_distance': struct.unpack('<H', data[1:3])[0] / 100.0,  # 最大检测距离，单位cm转m
                        'measurement_mode': data[3],                # 测量模式
                        'detection_zone': data[4],                  # 检测区域
                        'sensitivity': data[5],                     # 灵敏度设置
                        'update_rate': data[6],                     # 更新频率
                        'reserved': data[7]                         # 保留字节
                    })
                    
            elif message_type == 'TargetInformation':
                # 雷达目标信息解析 - 根据CAR28F应用手册0x70C消息格式
                if len(data) >= 8:
                    # 按照图片中的位域定义解析数据
                    # 将8字节数据转换为64位整数便于位操作
                    data_64bit = 0
                    for i in range(8):
                        data_64bit |= (data[i] << (8 * i))
                    
                    # 按照表格定义提取各字段
                    cluster_index = self.extract_bits_from_int(data_64bit, 0, 8)      # Bit 0-7
                    cluster_rcs_raw = self.extract_bits_from_int(data_64bit, 8, 8)    # Bit 8-15
                    cluster_range_raw = self.extract_bits_from_int(data_64bit, 16, 16) # Bit 16-31
                    cluster_azimuth_raw = self.extract_bits_from_int(data_64bit, 32, 8) # Bit 32-39
                    cluster_vrel_raw = self.extract_bits_from_int(data_64bit, 48, 11)  # Bit 48-58
                    cluster_rollcount = self.extract_bits_from_int(data_64bit, 46, 2)  # Bit 46-47
                    
                    # 按照新的距离计算公式: (RangeHValue*256 + RangeLValue)*0.01
                    range_l_value = cluster_range_raw & 0xFF        # 低8位
                    range_h_value = (cluster_range_raw >> 8) & 0xFF # 高8位
                    cluster_range = (range_h_value * 256 + range_l_value) * 0.01  # 距离计算
                    
                    # 按照计算公式转换为其他物理值
                    cluster_rcs = cluster_rcs_raw * 0.5 - 50        # RCS值：Val*0.5-50
                    cluster_azimuth = cluster_azimuth_raw - 90      # 角度：Val-90 (单位度)
                    cluster_vrel = cluster_vrel_raw * 0.05 - 35     # 速度：Val*0.05-35 (单位m/s)
                    
                    result.update({
                        # 原始值
                        'cluster_index': cluster_index,                # 目标ID (0-127)
                        'cluster_rcs_raw': cluster_rcs_raw,
                        'cluster_range_raw': cluster_range_raw,
                        'cluster_azimuth_raw': cluster_azimuth_raw,
                        'cluster_vrel_raw': cluster_vrel_raw,
                        'cluster_rollcount': cluster_rollcount,
                        
                        # 物理值
                        'target_id': cluster_index,                    # 目标ID
                        'rcs_value': cluster_rcs,                      # 雷达截面积 (dBm²)
                        'distance_m': cluster_range,                   # 距离 (m)
                        'azimuth_deg': cluster_azimuth,               # 方位角 (度)
                        'velocity_ms': cluster_vrel,                  # 相对速度 (m/s)
                        'roll_count': cluster_rollcount,              # 计数位
                        
                        # 额外计算的直角坐标
                        'pos_x': cluster_range * math.sin(math.radians(cluster_azimuth)),  # X坐标
                        'pos_y': cluster_range * math.cos(math.radians(cluster_azimuth))   # Y坐标
                    })
                    
                    # 判断目标类型
                    if abs(cluster_vrel) < 0.1:
                        target_type_name = '静止目标'
                    elif cluster_vrel > 0.1:
                        target_type_name = '远离目标'
                    elif cluster_vrel < -0.1:
                        target_type_name = '接近目标'
                    else:
                        target_type_name = '未知类型'
                    
                    result['target_type_name'] = target_type_name
                    
            elif message_type == 'RadarConfiguration':
                # 雷达配置消息解析（通常是发送给雷达的）
                if len(data) >= 8:
                    result.update({
                        'config_command': data[0],
                        'detection_range': struct.unpack('<H', data[1:3])[0],
                        'sensitivity_level': data[3],
                        'update_rate_config': data[4],
                        'angle_range': data[5],
                        'filter_settings': data[6],
                        'reserved_config': data[7]
                    })
            
            return result
            
        except Exception as e:
            print(f"解析CAR28F数据时出错: {e}")
            return {
                'error': str(e),
                'message_type': message_type if 'message_type' in locals() else 'unknown',
                'raw_data': data.hex(),
                'bytes': list(data)
            }
    
    def send_config_command(self, command_data):
        """
        发送配置命令到CAR28F传感器
        使用RadarConfiguration消息ID
        """
        try:
            config_id = self.message_ids['RadarConfiguration']
            message = can.Message(
                arbitration_id=config_id,
                data=command_data,
                is_extended_id=False
            )
            
            self.bus.send(message)
            print(f"发送配置命令: ID=0x{config_id:03X}, Data={command_data.hex()}")
            return True
            
        except Exception as e:
            print(f"发送配置命令失败: {e}")
            return False
    
    def read_sensor_data(self, timeout=1.0):
        """
        读取CAR28F传感器数据
        监听所有相关的消息ID
        
        Args:
            timeout: 接收超时时间(秒)
            
        Returns:
            解析后的传感器数据或None
        """
        if not self.bus:
            print("CAN总线未连接")
            return None
        
        try:
            # 接收CAN消息
            message = self.bus.recv(timeout=timeout)
            
            if message is None:
                return None
            
            # 检查是否是CAR28F相关的消息ID
            message_found = False
            message_name = "未知消息"
            
            for msg_name, msg_id in self.message_ids.items():
                if message.arbitration_id == msg_id:
                    message_found = True
                    message_name = msg_name
                    break
            
            if message_found:
                # 只处理和显示 0x70C (TargetInformation) 消息
                if message_name == 'TargetInformation':
                    # 解析数据
                    parsed_data = self.parse_car28f_data(message.arbitration_id, message.data)
                    
                    if 'error' not in parsed_data:
                        # 只输出目标ID和距离
                        target_id = parsed_data.get('target_id', 'N/A')
                        distance = parsed_data.get('distance_m', 'N/A')
                        if distance != 'N/A':
                            print(f"目标ID: {target_id}, 距离: {distance:.3f}m")
                        else:
                            print(f"目标ID: {target_id}, 距离: 解析失败")
                    
                    return {
                        'can_id': message.arbitration_id,
                        'message_name': message_name,
                        'timestamp': message.timestamp,
                        'dlc': message.dlc,
                        'raw_data': message.data,
                        'parsed_data': parsed_data
                    }
                else:
                    # 对于非 0x70C 消息，静默处理，不显示
                    return {
                        'can_id': message.arbitration_id,
                        'message_name': message_name,
                        'timestamp': message.timestamp,
                        'dlc': message.dlc,
                        'raw_data': message.data,
                        'parsed_data': {}
                    }
            else:
                # 显示其他ID的消息（用于调试）
                print(f"收到其他设备消息: 0x{message.arbitration_id:03X}, 数据: {message.data.hex()}")
                return None
                
        except can.CanError as e:
            print(f"CAN通信错误: {e}")
            return None
        except Exception as e:
            print(f"读取数据时发生错误: {e}")
            return None
    
    def initialize_sensor(self):
        """
        初始化CAR28F传感器
        发送配置命令启动雷达
        """
        print("正在初始化CAR28F雷达传感器...")
        
        # CAR28F配置命令示例
        # Byte 0: 命令码 (0x01=启动, 0x02=停止, 0x03=设置参数)
        # Byte 1-2: 检测距离 (cm, 小端序)
        # Byte 3: 灵敏度等级 (1-10)
        # Byte 4: 更新频率 (Hz)
        # Byte 5: 角度范围设置
        # Byte 6: 滤波设置
        # Byte 7: 保留字节
        
        # 启动雷达的配置命令
        start_command = bytes([
            0x01,           # 启动命令
            0xE8, 0x03,     # 检测距离 1000cm (小端序)
            0x05,           # 灵敏度等级 5
            0x14,           # 更新频率 20Hz
            0x78,           # 角度范围 ±60度
            0x01,           # 启用滤波
            0x00            # 保留字节
        ])
        
        if self.send_config_command(start_command):
            print("发送启动命令成功，等待雷达响应...")
            time.sleep(0.5)  # 等待传感器响应
            
            # 检查是否收到反馈
            feedback_received = False
            for _ in range(10):  # 尝试10次
                data = self.read_sensor_data(timeout=0.2)
                if data and data.get('message_name') == 'RadarFeedback':
                    print("收到雷达反馈，初始化成功!")
                    feedback_received = True
                    break
                time.sleep(0.1)
            
            if not feedback_received:
                print("警告: 未收到雷达反馈，但配置命令已发送")
            
            return True
        else:
            print("传感器初始化失败")
            return False
    
    def start_continuous_reading(self, read_interval=0.1):
        """
        开始连续读取CAR28F传感器数据
        
        Args:
            read_interval: 读取间隔(秒)
        """
        self.running = True
        print(f"开始连续读取CAR28F雷达数据...")
        
        # 统计计数器
        total_messages = 0
        start_time = time.time()
        
        try:
            while self.running:
                data = self.read_sensor_data(timeout=0.5)
                if data:
                    total_messages += 1
                
                time.sleep(read_interval)
                
        except KeyboardInterrupt:
            print("\n用户中断，停止读取")
        except Exception as e:
            print(f"连续读取时发生错误: {e}")
        finally:
            elapsed_time = time.time() - start_time
            print(f"\n运行时间: {elapsed_time:.2f} 秒")
            print(f"总接收消息: {total_messages}")
            if elapsed_time > 0:
                print(f"平均消息速率: {total_messages/elapsed_time:.2f} 条/秒")
    
    def disconnect(self):
        """断开CAN总线连接"""
        if self.bus:
            self.bus.shutdown()
            self.bus = None
            print("CAN总线连接已断开")
    
    def stop(self):
        """停止读取"""
        self.running = False
        self.disconnect()
    
    def analyze_target_distances(self, target_data_list):
        """
        分析目标距离数据
        
        Args:
            target_data_list: 目标数据列表
        """
        if not target_data_list:
            return
        
        print(f"\n=== 距离分析报告 ===")
        print(f"分析的目标数量: {len(target_data_list)}")
        
        distances = [data.get('distance_m', 0) for data in target_data_list if 'distance_m' in data]
        
        if distances:
            print(f"距离范围: {min(distances):.3f}m - {max(distances):.3f}m")
            print(f"平均距离: {sum(distances)/len(distances):.3f}m")
            
            # 距离分类统计
            close_targets = [d for d in distances if d < 5.0]
            medium_targets = [d for d in distances if 5.0 <= d < 20.0]
            far_targets = [d for d in distances if d >= 20.0]
            
            print(f"近距离目标 (<5m): {len(close_targets)} 个")
            print(f"中距离目标 (5-20m): {len(medium_targets)} 个")
            print(f"远距离目标 (≥20m): {len(far_targets)} 个")
            
            # 列出每个目标的距离
            print(f"\n各目标距离详情:")
            for i, data in enumerate(target_data_list):
                if 'distance_m' in data:
                    tid = data.get('target_id', i+1)
                    dist = data['distance_m']
                    angle = data.get('azimuth_deg', 0)
                    vel = data.get('velocity_ms', 0)
                    print(f"  目标{tid}: {dist:.3f}m (角度:{angle:.1f}°, 速度:{vel:.2f}m/s)")
        
        print("=" * 30)

def main():
    """主函数"""
    print("=== CAR28F雷达传感器CAN数据读取程序 ===")
    print("只输出目标ID和距离信息")
    
    # 创建CAR28F读取器实例
    can_reader = CAR28FSensorReader(
        interface='can0',      # CAN接口
        bitrate=500000,        # CAR28F可能使用500kbps或250kbps
        radar_id=0             # 雷达ID固定为0
    )
    
    # 设置CAN接口
    print("\n1. 设置CAN接口...")
    if not can_reader.setup_can_interface():
        print("CAN接口设置失败，程序退出")
        return
    
    # 连接CAN总线
    print("\n2. 连接CAN总线...")
    if not can_reader.connect():
        print("CAN总线连接失败，程序退出")
        return
    
    try:
        print("\n3. 开始监听雷达数据...")
        print("按 Ctrl+C 停止")
        print("-" * 30)
        can_reader.start_continuous_reading(read_interval=0.05)
    
    except Exception as e:
        print(f"程序执行时发生错误: {e}")
    
    finally:
        # 清理资源
        print("\n清理资源...")
        can_reader.stop()
        print("程序结束")

if __name__ == "__main__":
    main()