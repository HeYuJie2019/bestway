import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import math
import tf_transformations


class LivoxImuOdometryNode(Node):
    def __init__(self):
        super().__init__('livox_imu_odometry_node')

        # 订阅 /livox/imu 话题
        self.imu_subscription = self.create_subscription(
            Imu,
            '/livox/imu',
            self.imu_callback,
            10
        )

        # 发布 /odom 话题
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)

        # 初始化状态变量
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.last_time = self.get_clock().now()

        # 零偏校正
        self.ax_bias = 0.0
        self.ay_bias = 0.0
        self.wz_bias = 0.0
        self.bias_initialized = False

        # 滤波变量
        self.ax_filtered = 0.0
        self.ay_filtered = 0.0
        self.wz_filtered = 0.0

    def imu_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        if dt <= 0:
            return
        self.last_time = current_time

        # 初始化零偏
        if not self.bias_initialized:
            self.ax_bias = msg.linear_acceleration.x
            self.ay_bias = msg.linear_acceleration.y
            self.wz_bias = msg.angular_velocity.z
            self.bias_initialized = True
            self.get_logger().info("IMU 零偏已初始化")
            return

        # 获取加速度和角速度，减去零偏
        ax = msg.linear_acceleration.x - self.ax_bias
        ay = msg.linear_acceleration.y - self.ay_bias
        wz = msg.angular_velocity.z - self.wz_bias

        # 滤波
        self.ax_filtered = 0.9 * self.ax_filtered + 0.1 * ax
        self.ay_filtered = 0.9 * self.ay_filtered + 0.1 * ay
        self.wz_filtered = 0.9 * self.wz_filtered + 0.1 * wz

        ax = self.ax_filtered
        ay = self.ay_filtered
        wz = self.wz_filtered

        # 更新偏航角
        self.yaw += wz * dt

        # 转换到全局坐标系
        ax_global = ax * math.cos(self.yaw) - ay * math.sin(self.yaw)
        ay_global = ax * math.sin(self.yaw) + ay * math.cos(self.yaw)

        # 更新速度和位置
        decay_factor = 0.99
        self.vx = self.vx * decay_factor + ax_global * dt
        self.vy = self.vy * decay_factor + ay_global * dt
        self.x += self.vx * dt
        self.y += self.vy * dt

        # 发布里程计数据
        self.publish_odometry()

    def publish_odometry(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'

        # 设置位置
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # 设置姿态
        quaternion = tf_transformations.quaternion_from_euler(0, 0, self.yaw)
        odom.pose.pose.orientation = Quaternion(
            x=quaternion[0],
            y=quaternion[1],
            z=quaternion[2],
            w=quaternion[3]
        )

        # 设置速度
        odom.child_frame_id = 'base_link'
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.yaw

        self.odom_publisher.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = LivoxImuOdometryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()