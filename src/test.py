from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')

        # 储存odom的位姿
        self.odom_position = None
        self.odom_orientation = None

        # 订阅 /Odometry 话题
        self.odom_sub = self.create_subscription(
            Odometry,
            '/Odometry',
            self.odom_callback,
            1000
        )
        self.get_logger().info("订阅 /Odometry 话题")

    def odom_callback(self, msg):
        """
        处理 /Odometry 消息，保存位置和姿态
        """
        self.odom_position = msg.pose.pose.position
        self.odom_orientation = msg.pose.pose.orientation
        self.get_logger().info(
            f"收到Odom: pos=({self.odom_position.x:.3f}, {self.odom_position.y:.3f}, {self.odom_position.z:.3f}), "
            f"ori=({self.odom_orientation.x:.3f}, {self.odom_orientation.y:.3f}, {self.odom_orientation.z:.3f}, {self.odom_orientation.w:.3f})"
        )

def main(args=None):
    rclpy.init(args=args)

    test_node = TestNode()

    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()