#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import termios
import sys
import tty
import threading
import select

class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control_node')

        # 发布 /cmd_vel 话题
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # 初始化速度
        self.linear_speed = 3.0  # 默认线速度
        self.angular_speed = 3.0  # 默认角速度
        self.current_twist = Twist()  # 当前速度消息
        self.key_pressed = None  # 当前按下的键

        # 显示控制说明
        self.print_instructions()

        # 定时器，用于持续发布当前速度
        self.timer = self.create_timer(0.02, self.publish_twist)  # 每 0.1 秒发布一次

        # 启动键盘监听线程
        self.keyboard_thread = threading.Thread(target=self.run, daemon=True)
        self.keyboard_thread.start()

    def print_instructions(self):
        """
        打印控制说明
        """
        print("控制说明：")
        print("  w: 前进")
        print("  s: 后退")
        print("  a: 左转")
        print("  d: 右转")
        print("  i: 增大直线速度")
        print("  j: 减小直线速度")
        print("  o: 增大旋转角速度")
        print("  k: 减小旋转角速度")
        print("  按 Ctrl+C 退出")

    def run(self):
        """
        循环监听键盘输入
        """
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while rclpy.ok():
                if sys.stdin in select.select([sys.stdin], [], [], 0.1)[0]:  # 检查是否有按键输入
                    key = sys.stdin.read(1)
                    self.handle_key(key)
                else:
                    self.handle_key(None)  # 如果没有按键输入，传递 None
        except KeyboardInterrupt:
            print("退出键盘控制")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def handle_key(self, key):
        """
        根据键盘输入更新速度并发布消息
        """
        if key is None:  # 没有按键按下
            self.key_pressed = None
        elif key == 'w':  # 前进
            self.current_twist.linear.x = self.linear_speed
            self.current_twist.angular.z = 0.0
            self.key_pressed = 'w'
        elif key == 's':  # 后退
            self.current_twist.linear.x = -self.linear_speed
            self.current_twist.angular.z = 0.0
            self.key_pressed = 's'
        elif key == 'a':  # 左转
            self.current_twist.linear.x = 0.0
            self.current_twist.angular.z = self.angular_speed
            self.key_pressed = 'a'
        elif key == 'd':  # 右转
            self.current_twist.linear.x = 0.0
            self.current_twist.angular.z = -self.angular_speed
            self.key_pressed = 'd'
        elif key == 'i':  # 增大直线速度
            self.linear_speed += 0.1
            print(f"直线速度增加到: {self.linear_speed:.2f}")
        elif key == 'j':  # 减小直线速度
            self.linear_speed = max(0.0, self.linear_speed - 0.1)
            print(f"直线速度减少到: {self.linear_speed:.2f}")
        elif key == 'o':  # 增大旋转角速度
            self.angular_speed += 0.1
            print(f"旋转角速度增加到: {self.angular_speed:.2f}")
        elif key == 'k':  # 减小旋转角速度
            self.angular_speed = max(0.0, self.angular_speed - 0.1)
            print(f"旋转角速度减少到: {self.angular_speed:.2f}")
        elif key == '\x03':  # Ctrl+C
            raise KeyboardInterrupt

    def publish_twist(self):
        """
        定时发布当前速度
        """
        if self.key_pressed:
            # 如果有按键被按下，发布当前速度
            self.publisher.publish(self.current_twist)
            self.get_logger().info(f"发布速度: 线速度={self.current_twist.linear.x:.2f}, 角速度={self.current_twist.angular.z:.2f}")
        else:
            pass
            # 如果没有按键被按下，发布停止消息
            # self.current_twist.linear.x = 0.0
            # self.current_twist.angular.z = 0.0
            # self.publisher.publish(self.current_twist)
            # self.get_logger().info("发布停止消息: 线速度=0.0, 角速度=0.0")

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("节点已停止")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()