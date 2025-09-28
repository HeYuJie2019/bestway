import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from adafruit_servokit import ServoKit
from time import sleep


class ServoController:
    def __init__(self, channels=16):
        # 初始化 ServoKit
        self.kit = ServoKit(channels=channels)
        # 设置舵机的脉冲宽度范围
        # 500us ~ 2500us
        # 500us = 0.5ms
        # 2500us = 2.5ms
        self.kit.servo[0].set_pulse_width_range(500, 2500)
        self.kit.servo[1].set_pulse_width_range(500, 2500)
        # 设置舵机的初始角度
        self.kit.servo[0].angle = 90
        self.kit.servo[1].angle = 84

    def middle(self):
        """
        将两个舵机都设置为中间位置
        """
        self.set_angle(0, 90)
        self.set_angle(1, 84)

    def set_angle(self, servo_index, target_angle):
        """
        设置指定舵机的角度，并根据角度差添加延时
        :param servo_index: 舵机索引 (0 或 1)
        :param target_angle: 要设置的目标角度
        """
        if 0 <= servo_index < len(self.kit.servo):
            current_angle = self.kit.servo[servo_index].angle
            if current_angle is None:
                current_angle = 0  # 如果当前角度为 None，假设为 0

            # 计算角度差
            angle_difference = abs(target_angle - current_angle)

            # 设置舵机角度
            self.kit.servo[servo_index].angle = target_angle

            # 根据角度差延时，假设每移动 1 度延时 10 毫秒
            delay = angle_difference * 0.01
            # sleep(delay)

    def get_angle(self, servo_index):
        """
        获取指定舵机的角度
        :param servo_index: 舵机索引 (0 或 1)
        :return: 当前舵机的角度
        """
        if 0 <= servo_index < len(self.kit.servo):
            return self.kit.servo[servo_index].angle
        else:
            raise ValueError("舵机索引超出范围")
        
    def set_position(self, servo_index, position):
        """
        设置舵机的位置，使用线性映射将软件设定范围映射到舵机实际范围
        :param servo_index: 舵机索引 (0 或 1)
        :param position: 软件设定的位置 (0: -85~+85, 1: -90~+90)
        """
        if servo_index == 0:
            # 舵机 0 的软件范围是 -85 到 +85，实际范围是 5 到 175，中心是 90
            if position < -85 or position > 85:
                raise ValueError("舵机 0 的位置超出范围")
            # 映射公式：实际角度 = (位置 + 85) * (175 - 5) / (85 - (-85)) + 5
            target_angle = (-position + 85) * (175 - 5) / (85 - (-85)) + 5
            # 调整中心点为90
            target_angle = target_angle - 90 + 90
            self.set_angle(servo_index, target_angle)
        elif servo_index == 1:
            # 舵机 1 的软件范围是 -88 到 +88，实际范围是 0 到 168，中心是 84
            if position < -88 or position > 88:
                raise ValueError("舵机 1 的位置超出范围")
            # 映射公式：实际角度 = (位置 + 88) * (168 - 0) / (88 - (-88)) + 0
            target_angle = (position + 88) * (168 - 0) / (88 - (-88)) + 0
            # 调整中心点为84
            target_angle = target_angle - 84 + 84
            self.set_angle(servo_index, target_angle)
        else:
            raise ValueError("无效的舵机索引")

class ServoControllerNode(Node):
    def __init__(self):
        super().__init__('yuntai_controller_node')
        self.controller = ServoController()
        self.subscription = self.create_subscription(
            String,
            '/servo_position',
            self.listener_callback,
            10
        )
        self.get_logger().info("Servo Controller Node has been started.")

    def listener_callback(self, msg):
        """
        回调函数，处理接收到的舵机位置消息
        消息格式：'servo_index:position'，例如 '0:50' 或 '1:-30'
        """
        try:
            data = msg.data.split(':')
            servo_index = int(data[0])
            position = float(data[1])
            self.controller.set_position(servo_index, position)
            self.get_logger().info(f"Set servo {servo_index} to position {position}")
        except (ValueError, IndexError) as e:
            self.get_logger().error(f"Invalid message format: {msg.data}. Error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ServoControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Servo Controller Node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()