import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from livox_ros_driver2.msg import CustomMsg
import math

class PointCloudDistance(Node):
    def __init__(self):
        super().__init__('lidar_distance')
        self.subscription = self.create_subscription(
            CustomMsg,
            '/livox/lidar',  # 替换为你的点云话题名称
            self.listener_callback,
            10
        )
        self.publisher_ = self.create_publisher(Float32MultiArray, 'horizontal_distances', 10)

        # 提前计算分区参数，避免重复计算
        self.angle_min = -90.0  # 最左侧
        self.angle_max = 90.0   # 最右侧
        self.num_intervals = 21  # 将[-90°, 90°]分成21个区间
        self.interval_size = (self.angle_max - self.angle_min) / self.num_intervals

    def listener_callback(self, msg):
        # 初始化每个区间的点列表
        intervals = [[] for _ in range(self.num_intervals)]

        # 遍历点云数据，将点分配到对应的角度区间
        for point in msg.points:
            x, y, z = point.x, point.y, point.z

            # 提前过滤不需要的点
            if abs(z) >= 0.1 or x <= 0:
                continue

            # 计算点的水平角度和距离
            angle = math.atan2(y, x) * 57.2958  # 转换为角度（直接乘以 57.2958 代替 math.degrees）
            distance = (x**2 + y**2)**0.5

            # 计算点属于哪个角度区间
            interval_index = int((angle - self.angle_min) / self.interval_size)
            if 0 <= interval_index < self.num_intervals:
                intervals[interval_index].append(distance)

        # 计算每个区间的平均距离
        averaged_distances = [
            sum(interval) / len(interval) if interval else 0.0
            for interval in intervals
        ]

        # 发布结果
        distance_msg = Float32MultiArray()
        distance_msg.data = averaged_distances
        self.publisher_.publish(distance_msg)

        # 打印从右到左的全部距离（调试用）
        distances_str = ", ".join(f"{d:.2f}" for d in averaged_distances)
        self.get_logger().info(f'从右到左的距离: [{distances_str}]')

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudDistance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()