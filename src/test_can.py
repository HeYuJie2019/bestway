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
                # 雷达目标信息解析 - 这是主要的测量数据
                if len(data) >= 8:
                    distance_raw = struct.unpack('<H', data[0:2])[0]
                    velocity_raw = struct.unpack('<h', data[2:4])[0]  # 有符号
                    
                    result.update({
                        'distance_cm': distance_raw,                    # 距离，单位cm
                        'distance_m': distance_raw / 100.0,            # 距离，单位m
                        'velocity_cms': velocity_raw,                   # 速度，单位cm/s
                        'velocity_ms': velocity_raw / 100.0,           # 速度，单位m/s
                        'signal_strength': data[4],                     # 信号强度 0-255
                        'target_angle': struct.unpack('<h', data[5:7])[0] / 10.0,  # 角度，单位0.1度
                        'target_id': data[7] & 0x0F,                   # 目标ID (低4位)
                        'target_type': (data[7] & 0xF0) >> 4          # 目标类型 (高4位)
                    })
                    
                    # 目标类型解析
                    target_types = {
                        0: '无目标',
                        1: '静止目标',
                        2: '运动目标',
                        3: '接近目标',
                        4: '远离目标'
                    }
                    result['target_type_name'] = target_types.get(result['target_type'], '未知类型')
                    
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
                print(f"\n收到CAR28F消息: {message_name}")
                print(f"  CAN ID: 0x{message.arbitration_id:03X}")
                print(f"  数据长度: {message.dlc}")
                print(f"  原始数据: {message.data.hex()}")
                
                # 解析CAR28F专用数据
                parsed_data = self.parse_car28f_data(message.arbitration_id, message.data)
                print(f"  解析结果:")
                for key, value in parsed_data.items():
                    if key not in ['raw_data', 'bytes']:  # 避免重复显示原始数据
                        print(f"    {key}: {value}")
                
                return {
                    'can_id': message.arbitration_id,
                    'message_name': message_name,
                    'timestamp': message.timestamp,
                    'dlc': message.dlc,
                    'raw_data': message.data,
                    'parsed_data': parsed_data
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
        print("监听的消息类型:")
        for msg_name, msg_id in self.message_ids.items():
            print(f"  {msg_name}: 0x{msg_id:03X}")
        print("按 Ctrl+C 停止读取")
        
        # 统计计数器
        message_counters = {name: 0 for name in self.message_ids.keys()}
        total_messages = 0
        start_time = time.time()
        
        try:
            while self.running:
                data = self.read_sensor_data(timeout=0.5)
                if data:
                    total_messages += 1
                    message_name = data.get('message_name', 'unknown')
                    if message_name in message_counters:
                        message_counters[message_name] += 1
                    
                    # 特别处理目标信息消息
                    if message_name == 'TargetInformation':
                        parsed = data.get('parsed_data', {})
                        if 'distance_m' in parsed:
                            print(f"=== 目标检测数据 #{message_counters[message_name]} ===")
                            print(f"距离: {parsed['distance_m']:.2f}m")
                            print(f"速度: {parsed['velocity_ms']:.2f}m/s")
                            print(f"信号强度: {parsed['signal_strength']}")
                            print(f"角度: {parsed.get('target_angle', 'N/A')}°")
                            print(f"目标类型: {parsed.get('target_type_name', 'N/A')}")
                
                time.sleep(read_interval)
                
        except KeyboardInterrupt:
            print("\n用户中断，停止读取")
        except Exception as e:
            print(f"连续读取时发生错误: {e}")
        finally:
            elapsed_time = time.time() - start_time
            print(f"\n=== 统计信息 ===")
            print(f"运行时间: {elapsed_time:.2f} 秒")
            print(f"总接收消息: {total_messages}")
            print("各类型消息统计:")
            for msg_name, count in message_counters.items():
                if count > 0:
                    print(f"  {msg_name}: {count} 条")
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

def main():
    """主函数"""
    print("=== CAR28F雷达传感器CAN数据读取程序 ===")
    print("传感器型号: CAR28F")
    print("支持的消息类型:")
    print("  - RadarConfiguration (0x200) - 雷达配置")
    print("  - RadarFeedback (0x400) - 雷达回复") 
    print("  - RadarStatus (0x60A) - 雷达状态")
    print("  - TargetStatus (0x70B) - 目标状态")
    print("  - TargetInformation (0x70C) - 目标信息")
    
    # 获取雷达ID
    try:
        radar_id = int(input("\n请输入雷达ID (默认为0): ").strip() or "0")
    except ValueError:
        radar_id = 0
        print("输入无效，使用默认雷达ID: 0")
    
    # 创建CAR28F读取器实例
    can_reader = CAR28FSensorReader(
        interface='can0',      # CAN接口
        bitrate=500000,        # CAR28F可能使用500kbps或250kbps
        radar_id=radar_id      # 雷达ID
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
        # 选择运行模式
        print("\n请选择运行模式:")
        print("1. 初始化雷达并单次读取")
        print("2. 初始化雷达并连续读取")
        print("3. 仅监听数据 (不发送配置)")
        print("4. 扫描所有CAN ID (调试模式)")
        
        choice = input("请输入选择 (1, 2, 3 或 4): ").strip()
        
        if choice in ['1', '2']:
            # 初始化传感器
            print("\n3. 初始化CAR28F雷达...")
            can_reader.initialize_sensor()
            
            if choice == '1':
                print("\n4. 执行单次读取...")
                data = can_reader.read_sensor_data(timeout=5.0)
                if data:
                    print("读取成功!")
                else:
                    print("未接收到CAR28F雷达数据")
                    print("请检查雷达连接和CAN配置")
            else:
                print("\n4. 开始连续读取...")
                can_reader.start_continuous_reading(read_interval=0.05)
        
        elif choice == '3':
            print("\n3. 开始监听雷达数据...")
            can_reader.start_continuous_reading(read_interval=0.05)
        
        elif choice == '4':
            print("\n3. 扫描模式 - 监听所有CAN消息...")
            print("这将显示所有收到的CAN消息，用于调试")
            # 清空消息ID过滤，接收所有消息
            original_ids = can_reader.message_ids.copy()
            can_reader.message_ids = {}  # 清空过滤
            
            try:
                can_reader.start_continuous_reading(read_interval=0.01)
            finally:
                can_reader.message_ids = original_ids  # 恢复过滤
        
        else:
            print("无效选择，程序退出")
    
    except Exception as e:
        print(f"程序执行时发生错误: {e}")
    
    finally:
        # 清理资源
        print("\n5. 清理资源...")
        can_reader.stop()
        print("程序结束")

if __name__ == "__main__":
    main()