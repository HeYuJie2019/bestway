import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import time


class YuntaiSearchNode(Node):
    def __init__(self):
        super().__init__('yuntai_search_node')

        # 发布云台控制指令
        self.publisher = self.create_publisher(String, '/servo_position', 100)

        # 发布搜索模式状态
        # self.search_mode_publisher = self.create_publisher(Bool, '/search_mode_status', 100)

        # 云台当前角度
        self.current_horizontal_angle = 0
        self.current_vertical_angle = 0

        # 云台角度范围
        self.horizontal_angle_limit = 120
        self.vertical_angle_limit = 88

        # 搜索模式参数
        self.search_step = 30  # 每次转动的角度
        self.search_delay = 0.3  # 每次转动后的延时（秒）
        self.search_vertical_step = 10  # 垂直方向每次抬高的角度
        self.search_direction = 1  # 搜索方向（1：顺时针，-1：逆时针）

        # 搜索状态
        self.searching = True  # 直接开启搜索模式
        self.search_mode_msg = Bool()
        self.search_mode_msg.data = self.searching

        # 创建定时器来控制搜索运动
        self.search_timer = self.create_timer(self.search_delay, self.search_mode)
        
        # 创建定时器发布搜索状态
        # self.status_timer = self.create_timer(0.1, self.publish_search_mode_status)

        self.get_logger().info("Yuntai Search Node has been started.")
        self.get_logger().info("Starting search mode...")

    def publish_search_mode_status(self):
        """
        发布当前是否处于搜索模式的信息
        """
        self.search_mode_msg.data = self.searching
        self.search_mode_publisher.publish(self.search_mode_msg)

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
            
            self.get_logger().info(f"Yuntai position: Horizontal={horizontal_angle:.2f}, Vertical={vertical_angle:.2f}")
        except Exception as e:
            self.get_logger().error(f"Error controlling yuntai: {e}")

    def search_mode(self):
        """
        搜索模式：云台水平旋转一圈，若到达极限则抬高垂直角度继续搜索
        """
        # 检查当前角度是否已经在极限位置，如果是则重置搜索方向
        if self.current_horizontal_angle >= self.horizontal_angle_limit:
            self.search_direction = -1  # 如果已在右极限，向左搜索
            self.get_logger().info(f"当前角度{self.current_horizontal_angle}已达到右极限，设置向左搜索")
        elif self.current_horizontal_angle <= -self.horizontal_angle_limit:
            self.search_direction = 1   # 如果已在左极限，向右搜索
            self.get_logger().info(f"当前角度{self.current_horizontal_angle}已达到左极限，设置向右搜索")

        # 水平旋转
        if self.search_direction == 1 and self.current_horizontal_angle < self.horizontal_angle_limit:
            self.current_horizontal_angle += self.search_step

            if self.current_horizontal_angle >= self.horizontal_angle_limit:
                self.current_horizontal_angle = self.horizontal_angle_limit
                self.search_direction = -1  # 到达右极限，开始反向
                # 抬高垂直角度
                if self.current_vertical_angle < 10:
                    self.current_vertical_angle += self.search_vertical_step
                    self.get_logger().info(f"垂直角度抬高到: {self.current_vertical_angle}")
                else:
                    self.current_vertical_angle = 0
                    self.get_logger().info("垂直角度重置为0")
        
        elif self.search_direction == -1 and self.current_horizontal_angle > -self.horizontal_angle_limit:
            self.current_horizontal_angle -= self.search_step
            
            if self.current_horizontal_angle <= -self.horizontal_angle_limit:
                self.current_horizontal_angle = -self.horizontal_angle_limit
                self.search_direction = 1  # 到达左极限，开始正向
                # 抬高垂直角度
                if self.current_vertical_angle < 10:
                    self.current_vertical_angle += self.search_vertical_step
                    self.get_logger().info(f"垂直角度抬高到: {self.current_vertical_angle}")
                else:
                    self.current_vertical_angle = 0
                    self.get_logger().info("垂直角度重置为0")
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


def main(args=None):
    rclpy.init(args=args)
    node = YuntaiSearchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Yuntai Search Node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
