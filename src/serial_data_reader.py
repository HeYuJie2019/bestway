#!/usr/bin/env python3

import serial
import time
import sys
import threading
from datetime import datetime

class SerialDataReader:
    def __init__(self, port='/dev/ttyTHS1', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial_port = None
        self.running = False
        self.mode = 1  # 1: 原始数据, 2: 解析坐标数据
        
    def connect(self):
        """连接串口"""
        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1.0,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False
            )
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            print(f"✓ 串口连接成功: {self.port}")
            print(f"✓ 串口参数: {self.baudrate}bps, 8N1, 无流控")
            return True
        except Exception as e:
            print(f"✗ 串口连接失败: {e}")
            return False
    
    def set_mode(self, mode):
        """设置读取模式"""
        if mode in [1, 2]:
            self.mode = mode
            if mode == 1:
                print("📖 模式设置: 读取数据并展示")
            else:
                print("📖 模式设置: 读取数据解析后展示")
        else:
            print("❌ 无效模式，请选择 1 或 2")
    
    def parse_frames(self, data):
        """解析数据帧：帧头(0xAA55) + 数据 + 帧尾(0x0D0A)"""
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
    
    def process_raw_data(self, data):
        """处理原始数据模式"""
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        hex_str = ' '.join([f'{b:02X}' for b in data])
        ascii_str = ''.join([chr(b) if 32 <= b <= 126 else '.' for b in data])
        
        print(f"[{timestamp}] 接收数据 ({len(data)} 字节):")
        print(f"  HEX: {hex_str}")
        print(f"  ASCII: {ascii_str}")
        print("-" * 60)
    
    def process_coordinate_data(self, frame):
        """处理解析坐标数据模式"""
        if len(frame) < 4:
            return
        
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        
        # 提取数据部分（去掉帧头和帧尾）
        data_part = frame[2:-2]
        
        # 生成16进制字符串用于显示
        hex_str = ' '.join([f'{b:02X}' for b in frame])
        
        print(f"[{timestamp}] 完整帧 ({len(frame)} 字节): {hex_str}")
        
        # 检查是否为标准坐标帧格式 (16字节: 帧头2 + 坐标12 + 帧尾2)
        if len(frame) == 16 and len(data_part) == 12:
            # 坐标数据 (3个4字节坐标)
            try:
                x_val = int.from_bytes(data_part[0:4], byteorder='little', signed=True)
                y_val = int.from_bytes(data_part[4:8], byteorder='little', signed=True)
                z_val = int.from_bytes(data_part[8:12], byteorder='little', signed=True)
                
                # 转换回浮点数（除以100）
                x_coord = x_val / 100.0
                y_coord = y_val / 100.0
                z_coord = z_val / 100.0
                
                print(f"  🌐 坐标: X={x_coord:.2f}, Y={y_coord:.2f}, Z={z_coord:.2f}")
                print(f"  📊 原始值: X={x_val}, Y={y_val}, Z={z_val}")
                
            except Exception as e:
                print(f"  ❌ 解析坐标失败: {e}")
        
        elif len(data_part) == 1:
            # 可能是状态数据
            status_byte = data_part[0]
            print(f"  📡 状态数据: {status_byte} (0x{status_byte:02X})")
        
        else:
            print(f"  📦 其他数据 ({len(data_part)} 字节): {data_part.hex()}")
            print(f"  ℹ️  预期格式: 16字节坐标帧或5字节状态帧")
        
        print("-" * 60)
    
    def read_data(self):
        """读取串口数据"""
        if not self.serial_port:
            return
        
        print(f"🔍 开始监听串口数据... (模式: {self.mode})")
        print("按 Ctrl+C 停止监听")
        print("=" * 60)
        
        self.running = True
        
        try:
            while self.running:
                if self.serial_port.in_waiting > 0:
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    
                    if data:
                        if self.mode == 1:
                            # 原始数据模式
                            self.process_raw_data(data)
                        else:
                            # 解析坐标数据模式
                            frames = self.parse_frames(data)
                            for frame in frames:
                                self.process_coordinate_data(frame)
                
                time.sleep(0.01)  # 避免CPU占用过高
                
        except KeyboardInterrupt:
            print("\n🛑 用户中断，停止监听")
        except Exception as e:
            print(f"\n❌ 监听错误: {e}")
        finally:
            self.running = False
    
    def close(self):
        """关闭串口"""
        self.running = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            print("✓ 串口已关闭")

def print_help():
    """打印帮助信息"""
    print("="*70)
    print("             串口数据读取工具")
    print("="*70)
    print("功能说明:")
    print("  模式1: 读取数据并展示 - 显示所有接收到的原始字节数据")
    print("  模式2: 读取数据解析后展示 - 解析帧格式并提取坐标信息")
    print()
    print("数据格式说明:")
    print("  坐标帧: 帧头(2字节) + X坐标(4字节) + Y坐标(4字节) + Z坐标(4字节) + 帧尾(2字节)")
    print("  总长度: 16字节，坐标采用4字节有符号整数，小端序编码")
    print("  格式: AA 55 [X(4字节)] [Y(4字节)] [Z(4字节)] 0D 0A")
    print()
    print("使用方法:")
    print("  python3 serial_data_reader.py [端口] [波特率] [模式]")
    print("  例如: python3 serial_data_reader.py /dev/ttyTHS1 115200 2")
    print("="*70)

def main():
    # 默认参数
    port = '/dev/ttyTHS1'
    baudrate = 115200
    mode = 1
    
    # 解析命令行参数
    if len(sys.argv) >= 2:
        if sys.argv[1] in ['-h', '--help', 'help']:
            print_help()
            return
        port = sys.argv[1]
    
    if len(sys.argv) >= 3:
        try:
            baudrate = int(sys.argv[2])
        except ValueError:
            print("❌ 波特率必须是数字")
            return
    
    if len(sys.argv) >= 4:
        try:
            mode = int(sys.argv[3])
            if mode not in [1, 2]:
                print("❌ 模式必须是 1 或 2")
                return
        except ValueError:
            print("❌ 模式必须是数字")
            return
    
    # 显示启动信息
    print_help()
    print(f"📋 当前配置:")
    print(f"  端口: {port}")
    print(f"  波特率: {baudrate}")
    print(f"  模式: {mode} ({'读取数据并展示' if mode == 1 else '读取数据解析后展示'})")
    print()
    
    # 创建读取器
    reader = SerialDataReader(port, baudrate)
    reader.set_mode(mode)
    
    # 连接串口
    if not reader.connect():
        return
    
    try:
        # 交互式模式
        while True:
            print("\n📋 选择操作:")
            print("1. 开始读取数据")
            print("2. 切换到模式1 (读取数据并展示)")
            print("3. 切换到模式2 (读取数据解析后展示)")
            print("4. 更改串口设置")
            print("5. 帮助")
            print("6. 退出")
            
            choice = input("\n请选择 (1-6): ").strip()
            
            print(f"🔍 您选择了: {choice}")  # 调试信息
            
            if choice == '1':
                print("💡 开始读取数据...")
                reader.read_data()
            
            elif choice == '2':
                print("💡 切换到模式1...")
                reader.set_mode(1)
            
            elif choice == '3':
                print("💡 切换到模式2...")
                reader.set_mode(2)
            
            elif choice == '4':
                print("💡 更改串口设置...")
                print("\n📋 当前设置:")
                print(f"  端口: {reader.port}")
                print(f"  波特率: {reader.baudrate}")
                
                new_port = input(f"新端口 (当前: {reader.port}, 回车跳过): ").strip()
                if new_port:
                    print(f"💡 更改端口为: {new_port}")
                    reader.close()
                    reader.port = new_port
                    if not reader.connect():
                        break
                
                new_baudrate = input(f"新波特率 (当前: {reader.baudrate}, 回车跳过): ").strip()
                if new_baudrate:
                    try:
                        print(f"💡 更改波特率为: {new_baudrate}")
                        reader.close()
                        reader.baudrate = int(new_baudrate)
                        if not reader.connect():
                            break
                    except ValueError:
                        print("❌ 波特率必须是数字")
            
            elif choice == '5':
                print("💡 显示帮助...")
                print_help()
            
            elif choice == '6':
                print("💡 退出程序...")
                break
            
            else:
                print(f"❌ 无效选择: {choice}")
                print("💡 请输入 1-6 之间的数字")
    
    except KeyboardInterrupt:
        print("\n🛑 程序被中断")
    
    finally:
        reader.close()
        print("👋 程序结束")

if __name__ == '__main__':
    main()
