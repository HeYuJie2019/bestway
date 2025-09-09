#!/usr/bin/env python3
"""
简化版77GHz雷达距离显示程序
专注于显示目标距离信息
"""

import serial
import signal
import sys
import time
import os
import struct
from datetime import datetime

class RadarDistanceMonitor:
    def __init__(self, port='/dev/ttyCH341USB1', baudrate=921600):
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.running = True
        self.buffer = bytearray()
        
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
    
    def signal_handler(self, signum, frame):
        print("\n正在退出...")
        self.running = False
    
    def configure_serial(self):
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False
            )
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()
            return True
        except Exception as e:
            print(f"串口配置失败: {e}")
            return False
    
    def parse_distances(self, packet):
        try:
            header = b'\xCB\xFE\xDD\xFF'
            start_idx = packet.find(header)
            if start_idx == -1 or len(packet) < 8:
                return None
            
            data_count = struct.unpack('<I', packet[start_idx+4:start_idx+8])[0]
            if data_count == 0 or data_count > 20:
                return None
            
            distances = []
            for i in range(data_count):
                offset = start_idx + 8 + i * 20
                if offset + 20 > len(packet):
                    break
                
                item_data = packet[offset:offset + 20]
                signal_strength_raw = struct.unpack('<I', item_data[0:4])[0]
                distance_raw = struct.unpack('<I', item_data[4:8])[0]
                
                # 根据官方协议转换
                signal_strength = signal_strength_raw / 100.0
                distance_m = distance_raw / 100.0       # 原始值/100 = 米
                distance_cm = distance_m * 100.0        # 米转厘米用于显示
                
                distances.append({
                    'target': i + 1,
                    'distance_cm': distance_cm,
                    'distance_m': distance_m,
                    'signal_strength': signal_strength
                })
            
            return distances
            
        except Exception as e:
            return None
    
    def find_packets(self, new_data):
        self.buffer.extend(new_data) #原始值除以100后得到的是厘米
        packets = []
        
        while True:
            header = b'\xCB\xFE\xDD\xFF'
            start_idx = self.buffer.find(header)
            
            if start_idx == -1:
                if len(self.buffer) > 3:
                    self.buffer = self.buffer[-3:]
                break
            
            if start_idx > 0:
                self.buffer = self.buffer[start_idx:]
            
            if len(self.buffer) < 8:
                break
            
            try:
                data_count = struct.unpack('<I', self.buffer[4:8])[0]
                if data_count > 20 or data_count == 0:
                    self.buffer = self.buffer[1:]
                    continue
                
                packet_length = 8 + data_count * 20
                if len(self.buffer) >= packet_length:
                    packet = bytes(self.buffer[:packet_length])
                    packets.append(packet)
                    self.buffer = self.buffer[packet_length:]
                else:
                    break
            except:
                self.buffer = self.buffer[1:]
        
        return packets
    
    def display_distances(self, distances):
        if not distances:
            return
        
        # 找到距离最小的目标
        closest_target = min(distances, key=lambda x: x['distance_m'])
        
        distance_m = closest_target['distance_m']
        distance_cm = closest_target['distance_cm']
        signal = closest_target['signal_strength']
        
        # 根据距离(米)添加不同的标识
        if distance_m < 0.05:      # < 5cm
            status = "🔴 极近"
        elif distance_m < 0.20:    # < 20cm
            status = "🟠 很近"
        elif distance_m < 0.50:    # < 50cm
            status = "🟡 较近"
        elif distance_m < 1.00:    # < 1m
            status = "🟢 中等"
        else:
            status = "🔵 较远"
        
        print(f"\r⚡ [{datetime.now().strftime('%H:%M:%S')}] 最近目标: {status} 📏 {distance_m:6.3f}m ({distance_cm:5.1f}cm) 📶 {signal:5.1f}", end='', flush=True)
    
    def run(self):
        print("🎯 77GHz雷达最近距离监控程序")
        print(f"📡 设备: {self.port}")
        print(f"⚡ 波特率: {self.baudrate}")
        print("📏 只显示最近目标的距离信息")
        print("🎮 按 Ctrl+C 退出")
        print("=" * 60)
        
        if not os.path.exists(self.port):
            print(f"❌ 设备 {self.port} 不存在")
            return 1
        
        if not self.configure_serial():
            return 1
        
        print("✅ 串口配置成功，开始监控最近目标距离...")
        print()  # 空行，为实时更新做准备
        
        try:
            while self.running:
                if self.serial_conn.in_waiting > 0:
                    new_data = self.serial_conn.read(self.serial_conn.in_waiting)
                    packets = self.find_packets(new_data)
                    
                    for packet in packets:
                        distances = self.parse_distances(packet)
                        if distances:
                            self.display_distances(distances)
                else:
                    time.sleep(0.01)
                    
        except Exception as e:
            print(f"程序错误: {e}")
        finally:
            if self.serial_conn:
                self.serial_conn.close()
                print("串口已关闭")
        
        return 0

def main():
    try:
        import serial
    except ImportError:
        print("❌ 需要安装 pyserial: pip3 install pyserial")
        return 1
    
    monitor = RadarDistanceMonitor()
    return monitor.run()

if __name__ == "__main__":
    sys.exit(main())
