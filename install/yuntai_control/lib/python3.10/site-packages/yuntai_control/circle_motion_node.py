#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import time
import math

class CircleMotionNode(Node):
    def __init__(self):
        super().__init__('circle_motion_node')
        
        # 创建发布者，发布云台控制命令
        self.publisher_ = self.create_publisher(String, '/servo_position', 10)
        
        # 订阅温度话题
        self.temperature_subscription = self.create_subscription(
            Float32MultiArray,
            'temperature_matrix',
            self.temperature_callback,
            10)
        
        # 云台运动参数
        self.horizontal_range = 230.0  # 水平方向范围（度）
        self.vertical_range = 80.0     # 垂直方向范围（度）
        self.speed = 50.0              # 运动速度
        
        # 运动状态
        self.angle = 0.0               # 当前角度（0-360度）
        self.center_h = 0.0            # 水平中心位置
        self.center_v = 0.0            # 垂直中心位置
        
        # 温度数据相关
        self.last_temperature_time = 0  # 上次收到温度数据的时间
        self.rotation_step = 10.0       # 每次收到数据旋转的角度
        
        # 不再使用定时器自动旋转，而是根据温度数据驱动
        # self.timer = self.create_timer(0.5, self.move_timer_callback)
        
        self.get_logger().info('云台温度驱动旋转节点已启动')
        self.get_logger().info(f'水平旋转范围: ±{self.horizontal_range/2}度')
        self.get_logger().info(f'每次旋转角度: {self.rotation_step}度')
        self.get_logger().info('订阅话题: temperature_matrix')
        self.get_logger().info('发布话题: /servo_position (格式: servo_index:position)')
        self.get_logger().info('等待温度数据驱动旋转...')
        
    def temperature_callback(self, msg):
        """温度数据回调函数，每收到新的温度数据就旋转一次"""
        current_time = time.time()
        
        # 记录收到温度数据
        data_size = len(msg.data) if hasattr(msg, 'data') else 0
        self.get_logger().info(f'收到温度数据，大小: {data_size}，开始旋转')
        
        # 更新角度
        self.angle += self.rotation_step
        
        # 角度归一化到0-360度
        if self.angle >= 360.0:
            self.angle -= 360.0
            self.get_logger().info('完成一圈旋转，重新开始')
        
        # 执行旋转
        self.execute_rotation()
        
        # 更新时间记录
        self.last_temperature_time = current_time
    
    def execute_rotation(self):
        """执行旋转动作"""
        # 计算水平位置：在±115度范围内做正弦运动
        target_h = self.center_h + (self.horizontal_range / 2) * math.cos(math.radians(self.angle))
        
        # 垂直位置保持在中心位置
        target_v = self.center_v
        
        # 确保在舵机安全范围内
        # 舵机0（水平）: -123.56 到 +123.56
        target_h = max(-100, min(100, target_h))
        
        # 发送控制命令
        self.send_servo_commands(target_h, target_v)
        
        # 打印当前位置
        self.get_logger().info(f'旋转到位置: 水平={target_h:.1f}°, 当前角度={self.angle:.1f}°')
    
    def move_timer_callback(self):
        """定时器回调函数，计算并发布云台位置"""
        
        # 只做水平旋转运动
        # 水平位置：在±60度范围内做正弦运动（对应舵机0的软件范围约±123度）
        target_h = self.center_h + (self.horizontal_range / 2) * math.cos(math.radians(self.angle))
        
        # 垂直位置保持在中心位置，不做运动
        target_v = self.center_v
        
        # 确保在舵机安全范围内
        # 舵机0（水平）: -123.56 到 +123.56
        target_h = max(-100, min(100, target_h))
        
        # 发送控制命令
        self.send_servo_commands(target_h, target_v)
        
        # 更新角度，实现连续运动
        angle_increment = self.speed * 0.5  # 0.5秒间隔 * 速度
        self.angle += angle_increment
        
        # 角度归一化到0-360度
        if self.angle >= 360.0:
            self.angle -= 360.0
            self.get_logger().info('完成一圈水平旋转')
        
        # 打印当前位置（每隔一定角度打印一次）
        if int(self.angle) % 30 == 0:  # 每30度打印一次
            self.get_logger().info(f'水平位置: {target_h:.1f}°, 角度={self.angle:.1f}°')
    
    def send_servo_commands(self, horizontal, vertical):
        """发送舵机控制命令"""
        # 发送水平舵机命令（舵机索引0）
        h_msg = String()
        h_msg.data = f"0:{horizontal:.2f}"
        self.publisher_.publish(h_msg)
        
        # 只在第一次或者垂直位置改变时发送垂直舵机命令
        # 由于我们只做水平旋转，垂直位置始终为0，所以只需要发送一次
        if not hasattr(self, '_vertical_set') or self._vertical_set != vertical:
            v_msg = String()
            v_msg.data = f"1:{vertical:.2f}"
            self.publisher_.publish(v_msg)
            self._vertical_set = vertical
    
    def send_position_command(self, horizontal, vertical):
        """兼容旧版本的函数名"""
        self.send_servo_commands(horizontal, vertical)
    
    def set_motion_parameters(self, h_range=None, v_range=None, speed=None, rotation_step=None):
        """设置运动参数"""
        if h_range is not None:
            self.horizontal_range = min(180.0, max(10.0, h_range))
        if v_range is not None:
            self.vertical_range = min(60.0, max(10.0, v_range))
        if speed is not None:
            self.speed = min(90.0, max(5.0, speed))
        if rotation_step is not None:
            self.rotation_step = min(45.0, max(1.0, rotation_step))
            
        self.get_logger().info(f'运动参数已更新: 水平范围={self.horizontal_range}°, 垂直范围={self.vertical_range}°, 旋转步长={self.rotation_step}°')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CircleMotionNode()
        
        # 可以在这里修改运动参数
        # node.set_motion_parameters(h_range=120.0, v_range=40.0, rotation_step=15.0)
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print('\n收到中断信号，正在停止云台运动...')
    except Exception as e:
        print(f'发生错误: {e}')
    finally:
        if 'node' in locals():
            # 停止云台运动，回到中心位置
            node.send_servo_commands(0.0, 0.0)
            node.get_logger().info('云台已回到中心位置')
            node.destroy_node()
        
        rclpy.shutdown()

if __name__ == '__main__':
    main()
