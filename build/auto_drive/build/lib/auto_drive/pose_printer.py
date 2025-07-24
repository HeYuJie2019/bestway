#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class PosePrinterNode(Node):
    def __init__(self):
        super().__init__('pose_printer_node')
        self.subscription = self.create_subscription(
            Odometry,
            '/Odometry',
            self.odom_callback,
            10
        )

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.get_logger().info(
            f"位置: x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f} | "
            # f"四元数: x={ori.x:.3f}, y={ori.y:.3f}, z={ori.z:.3f}, w={ori.w:.3f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = PosePrinterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
