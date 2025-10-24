#!/usr/bin/env python3
"""
TF发布节点：从里程计发布odom->base_link变换
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToBaseLinkPublisher(Node):
    """发布odom到base_link的TF变换"""
    
    def __init__(self):
        super().__init__('odom_to_baselink_publisher')
        
        # 创建TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 订阅里程计
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/Odometry',
            self.odom_callback,
            10
        )
        
        self.get_logger().info('Odom to base_link TF publisher started')
        
    def odom_callback(self, msg: Odometry):
        """接收里程计并发布TF"""
        # 创建TF消息
        t = TransformStamped()
        
        # 设置时间戳和坐标系
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        # 设置位置
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        
        # 设置姿态
        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w
        
        # 发布TF
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    
    node = OdomToBaseLinkPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
