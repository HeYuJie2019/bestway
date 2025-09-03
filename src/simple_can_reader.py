#!/usr/bin/env python3
"""
简单CAN数据读取器 - 直接读取CAN总线数据并输出
专为CAR28F雷达传感器设计，可读取所有CAN消息
"""

import can
import time
import sys
import signal
import subprocess

class SimpleCANReader:
    def __init__(self, interface='can0', bitrate=500000):
        """
        初始化简单CAN读取器
        
        Args:
            interface: CAN接口名称 (默认: can0)
            bitrate: CAN总线比特率 (默认: 500000)
        """
        self.interface = interface
        self.bitrate = bitrate
        self.bus = None
        self.running = False
        
        # 设置信号处理器用于优雅退出
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
    
    def signal_handler(self, signum, frame):
        """信号处理器，用于优雅退出"""
        print(f"\n收到退出信号，正在关闭CAN连接...")
        self.stop()
        sys.exit(0)
    
    def setup_can_interface(self):
        """设置CAN接口"""
        try:
            print(f"正在设置CAN接口 {self.interface}...")
            
            # 关闭CAN接口
            subprocess.run(['sudo', 'ip', 'link', 'set', self.interface, 'down'], 
                          capture_output=True, check=False)
            
            # 设置CAN接口比特率
            subprocess.run(['sudo', 'ip', 'link', 'set', self.interface, 'type', 'can', 
                          'bitrate', str(self.bitrate)], 
                          capture_output=True, check=True)
            
            # 启动CAN接口
            subprocess.run(['sudo', 'ip', 'link', 'set', self.interface, 'up'], 
                          capture_output=True, check=True)
            
            print(f"✅ CAN接口设置成功: {self.interface} @ {self.bitrate} bps")
            return True
            
        except subprocess.CalledProcessError as e:
            print(f"❌ 设置CAN接口失败: {e}")
            return False
        except Exception as e:
            print(f"❌ 设置CAN接口时发生错误: {e}")
            return False
    
    def connect(self):
        """连接到CAN总线"""
        try:
            print(f"正在连接CAN总线...")
            self.bus = can.interface.Bus(
                channel=self.interface, 
                interface='socketcan',
                receive_own_messages=False
            )
            print(f"✅ 成功连接到CAN总线: {self.interface}")
            return True
            
        except Exception as e:
            print(f"❌ 连接CAN总线失败: {e}")
            print("请检查:")
            print("  1. CAN接口是否存在 (使用 'ip link show' 查看)")
            print("  2. 是否有足够的权限 (需要sudo权限)")
            print("  3. CAN设备是否正确连接")
            return False
    
    def format_message(self, message):
        """格式化CAN消息输出 - 简化版"""
        timestamp = time.strftime("%H:%M:%S", time.localtime(message.timestamp))
        
        # 提取目标ID (通常在Byte0和Byte1中)
        target_id = 0
        if len(message.data) >= 2:
            target_id = message.data[0]  # 简化版本，使用Byte0作为目标ID
        
        # 计算距离 (Byte2*256+Byte3)*0.01
        distance = 0.0
        if len(message.data) >= 4:
            byte2 = message.data[2]
            byte3 = message.data[3]
            distance = (byte2 * 256 + byte3) * 0.01
        
        # 简化输出格式
        output = f"[{timestamp}] 目标ID: {target_id:3d}, 距离: {distance:6.2f}m"
        
        return output
    
    def identify_message_type(self, can_id):
        """识别CAR28F消息类型"""
        message_types = {
            0x200: "雷达配置 (RadarConfiguration)",
            0x400: "雷达回复 (RadarFeedback)", 
            0x60A: "雷达状态 (RadarStatus)",
            0x70B: "目标状态 (TargetStatus)",
            0x70C: "目标信息 (TargetInformation)"
        }
        return message_types.get(can_id, None)
    
    def start_reading(self, filter_id=None, show_stats=True):
        """
        开始读取CAN数据
        
        Args:
            filter_id: 过滤特定的CAN ID (None表示接收所有消息)
            show_stats: 是否显示统计信息
        """
        self.running = True
        
        print(f"\n🚀 开始读取CAN数据...")
        if filter_id:
            print(f"📋 过滤CAN ID: 0x{filter_id:03X}")
        else:
            print(f"📋 接收所有CAN消息")
        print(f"⏹️  按 Ctrl+C 停止读取\n")
        
        # 统计信息和距离跟踪
        message_count = 0
        start_time = time.time()
        last_stats_time = start_time
        current_distances = {}  # 存储当前所有目标的距离
        last_output_time = start_time
        
        try:
            while self.running:
                # 接收CAN消息
                message = self.bus.recv(timeout=1.0)
                
                if message is None:
                    continue
                
                # 过滤消息
                if filter_id and message.arbitration_id != filter_id:
                    continue
                
                # 提取目标ID和距离
                if len(message.data) >= 4:
                    target_id = message.data[0]  # 使用Byte0作为目标ID
                    byte2 = message.data[2]
                    byte3 = message.data[3]
                    distance = (byte2 * 256 + byte3) * 0.01
                    
                    # 更新当前目标的距离
                    if distance > 0:  # 只记录有效距离
                        current_distances[target_id] = distance
                        
                        # 每次更新后找出最小距离并输出
                        if current_distances:
                            min_distance = min(current_distances.values())
                            min_target_id = min(current_distances, key=current_distances.get)
                            timestamp = time.strftime("%H:%M:%S", time.localtime(message.timestamp))
                            print(f"[{timestamp}] 目标{min_target_id} 最小距离: {min_distance:6.2f}m (共{len(current_distances)}个目标)")
                
                message_count += 1
                
                # 显示统计信息 (每30秒一次，减少干扰)
                if show_stats and time.time() - last_stats_time >= 30:
                    elapsed = time.time() - start_time
                    rate = message_count / elapsed if elapsed > 0 else 0
                    if current_distances:
                        min_dist = min(current_distances.values())
                        print(f"\n📊 统计: 已接收 {message_count} 条消息, 平均速率: {rate:.2f} 条/秒, 当前最小距离: {min_dist:.2f}m")
                    else:
                        print(f"\n📊 统计: 已接收 {message_count} 条消息, 平均速率: {rate:.2f} 条/秒")
                    last_stats_time = time.time()
                
        except KeyboardInterrupt:
            print(f"\n⏹️  用户中断读取")
        except Exception as e:
            print(f"\n❌ 读取过程中发生错误: {e}")
        finally:
            # 最终统计
            elapsed_time = time.time() - start_time
            rate = message_count / elapsed_time if elapsed_time > 0 else 0
            print(f"\n📊 最终统计:")
            print(f"  运行时间: {elapsed_time:.2f} 秒")
            print(f"  总消息数: {message_count} 条")
            print(f"  平均速率: {rate:.2f} 条/秒")
            if current_distances:
                min_distance = min(current_distances.values())
                min_target_id = min(current_distances, key=current_distances.get)
                print(f"  最终检测到 {len(current_distances)} 个目标")
                print(f"  最小距离: {min_distance:.2f}m (目标{min_target_id})")
            else:
                print(f"  未检测到有效距离数据")
    
    def read_single_message(self, timeout=5.0, filter_id=None):
        """
        读取单条CAN消息
        
        Args:
            timeout: 超时时间(秒)
            filter_id: 过滤特定的CAN ID
            
        Returns:
            True if message received, False otherwise
        """
        print(f"🔍 等待CAN消息...")
        if filter_id:
            print(f"📋 过滤CAN ID: 0x{filter_id:03X}")
        
        try:
            start_time = time.time()
            while time.time() - start_time < timeout:
                message = self.bus.recv(timeout=0.1)
                
                if message is None:
                    continue
                
                # 过滤消息
                if filter_id and message.arbitration_id != filter_id:
                    continue
                
                # 输出消息
                formatted_output = self.format_message(message)
                print(formatted_output)
                return True
            
            print(f"⏰ 超时: {timeout} 秒内未收到消息")
            return False
            
        except Exception as e:
            print(f"❌ 读取消息时发生错误: {e}")
            return False
    
    def list_active_ids(self, duration=10):
        """
        扫描活跃的CAN ID
        
        Args:
            duration: 扫描持续时间(秒)
        """
        print(f"🔍 扫描活跃的CAN ID ({duration}秒)...")
        
        active_ids = {}
        start_time = time.time()
        
        try:
            while time.time() - start_time < duration:
                message = self.bus.recv(timeout=0.1)
                
                if message is None:
                    continue
                
                can_id = message.arbitration_id
                if can_id not in active_ids:
                    active_ids[can_id] = 0
                active_ids[can_id] += 1
            
            print(f"\n📋 发现的活跃CAN ID:")
            for can_id, count in sorted(active_ids.items()):
                message_type = self.identify_message_type(can_id)
                type_info = f" ({message_type})" if message_type else ""
                print(f"  0x{can_id:03X}: {count} 条消息{type_info}")
            
            print(f"\n总计: {len(active_ids)} 个不同的CAN ID")
            
        except Exception as e:
            print(f"❌ 扫描过程中发生错误: {e}")
    
    def disconnect(self):
        """断开CAN总线连接"""
        if self.bus:
            self.bus.shutdown()
            self.bus = None
            print("🔌 CAN总线连接已断开")
    
    def stop(self):
        """停止读取"""
        self.running = False
        self.disconnect()

def main():
    """主函数"""
    print("=" * 60)
    print("🚗 CAR28F目标信息读取器")
    print("📡 直接读取0x70C目标信息消息")
    print("=" * 60)
    
    # 使用默认配置
    interface = "can0"
    bitrate = 500000
    
    print(f"使用默认配置:")
    print(f"  CAN接口: {interface}")
    print(f"  比特率: {bitrate} bps")
    print(f"  过滤ID: 0x70C (目标信息)")
    
    # 创建CAN读取器
    reader = SimpleCANReader(interface=interface, bitrate=bitrate)
    
    try:
        # 设置CAN接口
        if not reader.setup_can_interface():
            print("❌ CAN接口设置失败，程序退出")
            return
        
        # 连接CAN总线
        if not reader.connect():
            print("❌ CAN连接失败，程序退出")
            return
        
        # 直接开始读取CAR28F目标信息
        print(f"\n🚀 开始读取CAR28F目标信息 (0x70C)")
        print("⏹️  按 Ctrl+C 停止读取")
        reader.start_reading(filter_id=0x70C)
    
    except Exception as e:
        print(f"❌ 程序执行时发生错误: {e}")
    
    finally:
        # 清理资源
        print("\n🧹 清理资源...")
        reader.stop()
        print("👋 程序结束")

if __name__ == "__main__":
    main()
