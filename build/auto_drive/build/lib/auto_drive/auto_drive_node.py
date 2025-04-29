#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

class AutoDriveNode(Node):
    def __init__(self):
        super().__init__('auto_drive_node')

        # 发布 /cmd_vel 话题
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # 订阅激光雷达距离数据
        self.lidar_subscription = self.create_subscription(
            Float32MultiArray,
            'horizontal_distances',
            self.lidar_callback,
            10
        )

        # 初始化状态
        self.safe_distance = 1.0  # 安全距离，单位：米
        self.turning = False  # 是否正在转向

    def lidar_callback(self, msg):
        """
        激光雷达数据回调函数
        """
        distances = msg.data  # 获取水平距离数组
        num_points = len(distances)

        # 将水平距离数组分为左、中、右三个区域
        left_distances = distances[:num_points // 3]
        front_distances = distances[num_points // 3:num_points * 2 // 3]
        right_distances = distances[num_points * 2 // 3:]

        # 计算每个区域的平均距离
        left_avg = sum(left_distances) / len(left_distances)
        front_avg = sum(front_distances) / len(front_distances)
        right_avg = sum(right_distances) / len(right_distances)

        self.get_logger().info(
            f"左侧平均距离: {left_avg:.2f} 米, 前方平均距离: {front_avg:.2f} 米, 右侧平均距离: {right_avg:.2f} 米"
        )

        # 根据距离判断最优方向
        if front_avg > self.safe_distance:
            # 前方安全，继续前进
            self.drive_forward()
        elif left_avg > right_avg:
            # 左侧更安全，左转
            self.turn_left()
        else:
            # 右侧更安全，右转
            self.turn_right()

    def drive_forward(self):
        """
        前进
        """
        if not self.turning:
            twist = Twist()
            twist.linear.x = 0.2  # 设置前进速度
            twist.angular.z = 0.0
            self.cmd_vel_publisher.publish(twist)
            self.get_logger().info("前进中...")

    def turn_left(self):
        """
        左转
        """
        self.turning = True
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.5  # 设置左转角速度
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info("左转中...")
        self.create_timer(1.0, self.resume_forward)  # 转向 1 秒后恢复前进

    def turn_right(self):
        """
        右转
        """
        self.turning = True
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = -0.5  # 设置右转角速度
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info("右转中...")
        self.create_timer(1.0, self.resume_forward)  # 转向 1 秒后恢复前进

    def resume_forward(self):
        """
        恢复前进
        """
        self.turning = False
        self.drive_forward()

def main(args=None):
    rclpy.init(args=args)
    node = AutoDriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()