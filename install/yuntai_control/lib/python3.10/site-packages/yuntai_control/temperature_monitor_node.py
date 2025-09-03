#!/usr/bin/env python3
"""
温度监控节点
专门用于订阅温度话题并将数据保存到共享存储中
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time
import numpy as np
from .temperature_data_storage import get_temperature_storage

class TemperatureMonitorNode(Node):
    """温度监控节点"""
    
    def __init__(self):
        super().__init__('temperature_monitor_node')
        
        # 获取温度数据存储实例
        self.temperature_storage = get_temperature_storage()
        
        # 创建温度话题订阅者
        self.temperature_subscription = self.create_subscription(
            Float32MultiArray,
            'temperature_matrix',
            self.temperature_callback,
            10
        )
        
        # 统计信息
        self.message_count = 0
        self.start_time = time.time()
        self.last_log_time = time.time()
        self.log_interval = 10.0  # 每10秒输出一次统计信息
        
        # 温度阈值设置
        self.high_temp_threshold = 50.0  # 高温阈值
        self.low_temp_threshold = 0.0    # 低温阈值
        
        self.get_logger().info("温度监控节点已启动")
        self.get_logger().info("开始监控温度话题: temperature_matrix")
        
    def temperature_callback(self, msg):
        """温度数据回调函数"""
        try:
            # 获取矩阵尺寸
            if len(msg.layout.dim) >= 2:
                rows = msg.layout.dim[0].size
                cols = msg.layout.dim[1].size
            else:
                # 如果没有维度信息，尝试推断
                data_size = len(msg.data)
                rows = int(np.sqrt(data_size))
                cols = rows
                self.get_logger().warn(f"未找到矩阵维度信息，推断为 {rows}x{cols}")
            
            # 更新温度数据到共享存储
            self.temperature_storage.update_temperature_data(msg.data, rows, cols)
            
            # 更新统计信息
            self.message_count += 1
            current_time = time.time()
            
            # 计算当前温度统计
            if msg.data:
                # 将array.array转换为列表进行处理
                data_list = list(msg.data)
                max_temp = max(data_list)
                min_temp = min(data_list)
                avg_temp = sum(data_list) / len(data_list)
                
                # 检查温度异常
                self.check_temperature_alerts(max_temp, min_temp)
                
                # 定期输出统计信息
                if current_time - self.last_log_time >= self.log_interval:
                    self.log_statistics(max_temp, min_temp, avg_temp, current_time)
                    self.last_log_time = current_time
            
        except Exception as e:
            self.get_logger().error(f"处理温度数据时发生错误: {e}")
            import traceback
            self.get_logger().error(f"错误详情: {traceback.format_exc()}")
    
    def check_temperature_alerts(self, max_temp, min_temp):
        """检查温度警报"""
        if max_temp > self.high_temp_threshold:
            self.get_logger().warn(f"检测到高温: {max_temp:.2f}°C (阈值: {self.high_temp_threshold}°C)")
        
        if min_temp < self.low_temp_threshold:
            self.get_logger().warn(f"检测到低温: {min_temp:.2f}°C (阈值: {self.low_temp_threshold}°C)")
    
    def log_statistics(self, max_temp, min_temp, avg_temp, current_time):
        """输出统计信息"""
        elapsed_time = current_time - self.start_time
        message_rate = self.message_count / elapsed_time if elapsed_time > 0 else 0
        
        # 获取存储的统计信息
        storage_stats = self.temperature_storage.get_statistics()
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("📊 温度监控统计信息")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"📈 当前温度: 最高={max_temp:.2f}°C, 最低={min_temp:.2f}°C, 平均={avg_temp:.2f}°C")
        self.get_logger().info(f"📊 累计统计: 平均最高={storage_stats.get('avg_max_temp', 0):.2f}°C, 平均最低={storage_stats.get('avg_min_temp', 0):.2f}°C")
        self.get_logger().info(f"📦 接收消息: {self.message_count} 条")
        self.get_logger().info(f"📈 消息速率: {message_rate:.2f} 条/秒")
        self.get_logger().info(f"⏱️  运行时间: {elapsed_time:.2f} 秒")
        self.get_logger().info(f"💾 数据年龄: {storage_stats.get('data_age', 0):.2f} 秒")
        self.get_logger().info("=" * 60)
    
    def set_temperature_thresholds(self, high_threshold, low_threshold):
        """设置温度阈值"""
        self.high_temp_threshold = high_threshold
        self.low_temp_threshold = low_threshold
        self.get_logger().info(f"温度阈值已更新: 高温={high_threshold}°C, 低温={low_threshold}°C")

def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        # 创建温度监控节点
        temperature_monitor = TemperatureMonitorNode()
        
        # 设置温度阈值（可根据需要调整）
        temperature_monitor.set_temperature_thresholds(high_threshold=60.0, low_threshold=-10.0)
        
        print("🌡️  温度监控节点已启动")
        print("📡 正在监控温度话题: temperature_matrix")
        print("💾 温度数据将保存到共享存储中")
        print("⏹️  按 Ctrl+C 停止监控")
        
        # 运行节点
        rclpy.spin(temperature_monitor)
        
    except KeyboardInterrupt:
        print("\n⏹️  用户中断，正在停止温度监控...")
    except Exception as e:
        print(f"❌ 运行温度监控节点时发生错误: {e}")
    finally:
        # 清理资源
        if 'temperature_monitor' in locals():
            temperature_monitor.destroy_node()
        rclpy.shutdown()
        print("👋 温度监控节点已停止")

if __name__ == '__main__':
    main()
