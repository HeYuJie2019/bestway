import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
import numpy as np


class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, error, dt):
        """
        计算 PID 输出
        :param error: 当前误差
        :param dt: 时间间隔
        :return: PID 控制输出
        """
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative


class TemperatureTrackingNode(Node):
    def __init__(self):
        super().__init__('temperature_tracking_node')

        # 订阅温度矩阵消息
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'temperature_matrix',
            self.temperature_callback,
            10
        )

        # 发布云台控制指令
        self.publisher = self.create_publisher(String, '/servo_position', 10)

        # 初始化 PID 控制器
        self.horizontal_pid = PIDController(kp=0.5, ki=0.01, kd=0.1)
        self.vertical_pid = PIDController(kp=0.5, ki=0.01, kd=0.1)

        # 云台当前角度
        self.current_horizontal_angle = 0
        self.current_vertical_angle = 0

        # 上次回调时间
        self.last_time = self.get_clock().now()

        self.get_logger().info("Temperature Tracking Node with PID has been started.")

    def temperature_callback(self, msg):
        """
        处理温度矩阵消息，计算最高温度的位置，并通过 PID 控制云台
        """
        try:
            # 获取温度矩阵的维度
            rows = msg.layout.dim[0].size
            cols = msg.layout.dim[1].size

            # 将温度数据转换为 NumPy 数组
            temperature_matrix = np.array(msg.data).reshape((rows, cols))

            # 找到最高温度的位置
            max_temp = np.max(temperature_matrix)
            max_index = np.unravel_index(np.argmax(temperature_matrix), temperature_matrix.shape)
            max_row, max_col = max_index

            # 计算目标位置（矩阵中心）
            target_row = rows // 2
            target_col = cols // 2

            # 计算误差
            horizontal_error = target_col - max_col  # 水平误差
            vertical_error = target_row - max_row  # 垂直误差

            # 获取当前时间并计算时间间隔
            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds / 1e9  # 转换为秒
            self.last_time = current_time

            # 通过 PID 控制计算角度调整
            horizontal_adjustment = self.horizontal_pid.compute(horizontal_error, dt)
            vertical_adjustment = self.vertical_pid.compute(vertical_error, dt)

            # 更新云台当前角度
            self.current_horizontal_angle += horizontal_adjustment
            self.current_vertical_angle += vertical_adjustment

            # 限制角度范围
            self.current_horizontal_angle = max(-132, min(132, self.current_horizontal_angle))
            self.current_vertical_angle = max(-88, min(88, self.current_vertical_angle))

            # 发布云台控制指令
            self.control_yuntai(self.current_horizontal_angle, self.current_vertical_angle)

            # 打印最高温度及其方向
            self.get_logger().info(f"Max Temp: {max_temp:.2f} at ({max_row}, {max_col}), "
                                   f"Horizontal Angle: {self.current_horizontal_angle:.2f}, "
                                   f"Vertical Angle: {self.current_vertical_angle:.2f}")
        except Exception as e:
            self.get_logger().error(f"Error processing temperature matrix: {e}")

    def control_yuntai(self, horizontal_angle, vertical_angle):
        """
        控制云台转向指定的角度
        :param horizontal_angle: 水平角度
        :param vertical_angle: 垂直角度
        """
        try:
            # 控制水平舵机
            horizontal_msg = String()
            horizontal_msg.data = f"0:{horizontal_angle}"
            self.publisher.publish(horizontal_msg)

            # 控制垂直舵机
            vertical_msg = String()
            vertical_msg.data = f"1:{vertical_angle}"
            self.publisher.publish(vertical_msg)
        except Exception as e:
            self.get_logger().error(f"Error controlling yuntai: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TemperatureTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Temperature Tracking Node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()