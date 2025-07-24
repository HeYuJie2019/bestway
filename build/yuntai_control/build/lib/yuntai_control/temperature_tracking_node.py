import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String, Bool, Int32, Float32
import numpy as np
import time


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
        self.publisher = self.create_publisher(String, '/servo_position', 100)

        # 发布搜索模式状态
        self.search_mode_publisher = self.create_publisher(Bool, '/search_mode_status', 100)

        # 发布大于 1000 的温度计数值
        self.count_publisher = self.create_publisher(Int32, '/count_above_1000', 10)

        # 云台当前角度
        self.current_horizontal_angle = 0
        self.current_vertical_angle = 0

        # 云台角度范围
        self.horizontal_angle_limit = 120
        self.vertical_angle_limit = 88

        # 最大步长
        self.max_step_size = 10  # 最大每次调整的角度

        # 温差阈值
        self.temperature_threshold = 400

        # 搜索模式参数
        self.search_step = 30  # 每次转动的角度
        self.search_delay = 0.3  # 每次转动后的延时（秒）
        self.search_vertical_step = 10  # 垂直方向每次抬高的角度
        self.search_direction = 1 # 搜索方向（1：顺时针，-1：逆时针）

        # 搜索状态
        self.searching = False  # 是否处于搜索模式
        self.search_mode_msg = Bool()
        self.search_mode_msg.data = self.searching
        self.timer = self.create_timer(0.1, self.publish_search_mode_status)

        # 发布温度均值
        self.average_temperature_publisher = self.create_publisher(Float32, '/average_temperature', 10)

        # 用于存储一圈的温度数据
        self.temperature_data = []


        self.get_logger().info("Temperature Tracking Node has been started.")

    def publish_search_mode_status(self):
        """
        发布当前是否处于搜索模式的信息
        """
        self.search_mode_msg.data = self.searching
        self.search_mode_publisher.publish(self.search_mode_msg)

    def temperature_callback(self, msg):
        """
        处理温度矩阵消息，计算最高温度的位置，并动态调整云台角度
        """
        try:
            # 获取温度矩阵的维度
            rows = msg.layout.dim[0].size
            cols = msg.layout.dim[1].size

            # 将温度数据转换为 NumPy 数组
            self.temperature_matrix = np.array(msg.data).reshape((rows, cols))
            # self.get_logger().info(f"Received temperature matrix: {temperature_matrix}")

            # 找到最高温度和最低温度
            max_temp = np.max(self.temperature_matrix)
            non_zero_values = self.temperature_matrix[self.temperature_matrix > 0]
            if non_zero_values.size > 0:
                min_temp = np.min(non_zero_values)
            else:
                min_temp = 0  # 或者可以设为None或其他默认值

            # 计算大于 1000 的温度计数
            count_above_1000 = int(np.sum(self.temperature_matrix > 1000))

            # 发布大于 1000 的温度计数
            count_msg = Int32()
            count_msg.data = count_above_1000
            self.count_publisher.publish(count_msg)

            self.get_logger().info(f"Max Temp: {max_temp:.2f}, Min Temp: {min_temp:.2f}, count_above_1000: {count_above_1000}")

            # 判断温差是否达到阈值
            if max_temp - min_temp < self.temperature_threshold:
                # 如果温差不足，进入搜索模式
                if not self.searching:  # 只有在非搜索模式时才打印进入搜索的信息
                    self.get_logger().info(f"温差{max_temp - min_temp:.2f}小于阈值{self.temperature_threshold}，进入搜索模式")
                self.searching = True
                self.temp_enough_time = None  # 重置计时
                self.search_mode()
                return
            
            # 如果温差足够，需持续超过1秒才退出搜索模式
            now = time.time()
            if not hasattr(self, 'temp_enough_time') or self.temp_enough_time is None:
                self.temp_enough_time = now
            elif now - self.temp_enough_time >= 0.1:
                if self.searching:
                    self.searching = False

            # 找到最高温度的位置
            max_temp = np.max(self.temperature_matrix)
            max_index = np.unravel_index(np.argmax(self.temperature_matrix), self.temperature_matrix.shape)
            max_row, max_col = max_index

            # 计算目标位置（矩阵中心）
            target_row = rows // 2
            target_col = cols // 2

            # 计算误差
            horizontal_error = target_col - max_col  # 水平误差
            vertical_error = target_row - max_row  # 垂直误差

            # 动态调整步长
            horizontal_step = self.calculate_step_size(horizontal_error, cols)
            vertical_step = self.calculate_step_size(vertical_error, rows)

            # 根据最高温度点的位置调整云台
            if horizontal_error > 0:  # 在中心点左边
                self.current_horizontal_angle += horizontal_step
            elif horizontal_error < 0:  # 在中心点右边
                self.current_horizontal_angle -= horizontal_step

            if vertical_error > 0:  # 在中心点上方
                self.current_vertical_angle -= vertical_step
            elif vertical_error < 0:  # 在中心点下方
                self.current_vertical_angle += vertical_step

            # 限制角度范围
            self.current_horizontal_angle = max(-self.horizontal_angle_limit, min(self.horizontal_angle_limit, self.current_horizontal_angle))
            self.current_vertical_angle = max(-self.vertical_angle_limit, min(self.vertical_angle_limit, self.current_vertical_angle))

            # 发布云台控制指令
            self.control_yuntai(self.current_horizontal_angle, self.current_vertical_angle)

            # 发布均温
            self.average_temperature_publisher.publish(Float32(data=float(np.mean(non_zero_values))))
            self.get_logger().info(f"Published average temperature: {np.mean(non_zero_values):.2f}")

            # 打印最高温度及其方向
            self.get_logger().info(f"Max Temp: {max_temp:.2f} at ({max_row}, {max_col}), "
                                   f"Horizontal Angle: {self.current_horizontal_angle:.2f}, "
                                   f"Vertical Angle: {self.current_vertical_angle:.2f}")
        except Exception as e:
            self.get_logger().error(f"Error processing temperature matrix: {e}")

    def calculate_step_size(self, error, dimension_size):
        """
        根据误差动态计算步长
        :param error: 当前误差
        :param dimension_size: 矩阵的维度大小（行或列）
        :return: 动态步长
        """
        normalized_error = abs(error) / (dimension_size // 2)  # 归一化误差（0 到 1）
        step_size = normalized_error * self.max_step_size  # 根据误差比例计算步长
        return max(1, step_size)  # 最小步长为 1

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

    def search_mode(self):
        """
        搜索模式：云台水平旋转一圈，若未找到目标则抬高垂直角度继续搜索
        """
        if not self.searching:
            self.get_logger().info("Entering search mode...")
            self.searching = True
            self.temperature_data = []  # 清空上一圈的温度数据
            
            # 检查当前角度是否已经在极限位置，如果是则重置搜索方向
            if self.current_horizontal_angle >= self.horizontal_angle_limit:
                self.search_direction = -1  # 如果已在右极限，向左搜索
                self.get_logger().info(f"当前角度{self.current_horizontal_angle}已达到右极限，设置向左搜索")
            elif self.current_horizontal_angle <= -self.horizontal_angle_limit:
                self.search_direction = 1   # 如果已在左极限，向右搜索
                self.get_logger().info(f"当前角度{self.current_horizontal_angle}已达到左极限，设置向右搜索")
            else:
                # 如果在中间位置，优先向右搜索（正方向）
                self.search_direction = 1
                self.get_logger().info(f"当前角度{self.current_horizontal_angle}在中间位置，设置向右搜索")

        # 水平旋转一圈
        if self.search_direction == 1 and self.current_horizontal_angle < self.horizontal_angle_limit:
            self.current_horizontal_angle += self.search_step

            # 温度矩阵数据
            current_temperature_matrix = self.temperature_matrix
            non_zero_values = current_temperature_matrix[current_temperature_matrix > 0]
            if non_zero_values.size > 0:
                self.temperature_data.append(np.mean(non_zero_values))

            if self.current_horizontal_angle >= self.horizontal_angle_limit:
                self.current_horizontal_angle = self.horizontal_angle_limit
                self.search_direction = -1  # 到达右极限，开始反向
                # 抬高垂直角度
                if self.current_vertical_angle < 10:
                    self.current_vertical_angle += self.search_vertical_step
                else:
                    self.current_vertical_angle = 0
                if self.temperature_data:
                    average_temperature = float(np.mean(self.temperature_data))
                    avg_temp_msg = Float32()
                    avg_temp_msg.data = average_temperature
                    self.average_temperature_publisher.publish(avg_temp_msg)
                    self.get_logger().info(f"Published average temperature for one rotation: {average_temperature:.2f}")
        
        elif self.search_direction == -1 and self.current_horizontal_angle > -self.horizontal_angle_limit:
            self.current_horizontal_angle -= self.search_step

            # 温度矩阵数据
            current_temperature_matrix = self.temperature_matrix
            non_zero_values = current_temperature_matrix[current_temperature_matrix > 0]
            if non_zero_values.size > 0:
                self.temperature_data.append(np.mean(non_zero_values))
            
            if self.current_horizontal_angle <= -self.horizontal_angle_limit:
                self.current_horizontal_angle = -self.horizontal_angle_limit
                self.search_direction = 1  # 到达左极限，开始正向
                # 抬高垂直角度
                if self.current_vertical_angle < 10:
                    self.current_vertical_angle += self.search_vertical_step
                else:
                    self.current_vertical_angle = 0
                if self.temperature_data:
                    average_temperature = float(np.mean(self.temperature_data))
                    avg_temp_msg = Float32()
                    avg_temp_msg.data = average_temperature
                    self.average_temperature_publisher.publish(avg_temp_msg)
                    self.get_logger().info(f"Published average temperature for one rotation: {average_temperature:.2f}")
        else:
            # 如果两个条件都不满足，说明可能卡在某个位置，强制重置
            self.get_logger().warn(f"搜索模式可能卡住: 当前角度={self.current_horizontal_angle}, 搜索方向={self.search_direction}")
            if self.current_horizontal_angle >= self.horizontal_angle_limit:
                self.search_direction = -1
                self.get_logger().info("强制设置向左搜索")
            elif self.current_horizontal_angle <= -self.horizontal_angle_limit:
                self.search_direction = 1
                self.get_logger().info("强制设置向右搜索")

        # 限制角度范围
        self.current_horizontal_angle = max(-self.horizontal_angle_limit, min(self.horizontal_angle_limit, self.current_horizontal_angle))
        self.current_vertical_angle = max(-10, min(10, self.current_vertical_angle))

        # 发布云台控制指令
        self.control_yuntai(self.current_horizontal_angle, self.current_vertical_angle)

        # 延时以放慢搜索速度
        # time.sleep(self.search_delay)

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