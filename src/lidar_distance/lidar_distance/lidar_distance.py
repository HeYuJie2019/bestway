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

    def listener_callback(self, msg):
        # 定义角度范围（假设雷达正前方的角度范围为[-90°, 90°]）
        angle_min = -90.0  # 最左侧
        angle_max = 90.0   # 最右侧
        num_intervals = 21 # 将[-90°, 90°]分成21个区间
        interval_size = (angle_max - angle_min) / num_intervals

        # 初始化每个区间的点列表
        intervals = [[] for _ in range(num_intervals)]

        # 遍历点云数据，将点分配到对应的角度区间
        for point in msg.points:
            x, y, z = point.x, point.y, point.z
            if abs(z) < 0.1 and x > 0:  # 仅考虑接近水平面的点
                # 计算点的水平角度（以度为单位）
                angle = math.degrees(math.atan2(y, x))
                # 计算点属于哪个角度区间
                interval_index = int((angle - angle_min) / interval_size)
                if 0 <= interval_index < num_intervals:
                    distance = (x**2 + y**2)**0.5
                    intervals[interval_index].append(distance)

        # 计算每个区间的平均距离
        averaged_distances = []
        for interval_points in intervals:
            if interval_points:
                avg_distance = sum(interval_points) / len(interval_points)
            else:
                avg_distance = 0.0  # 如果区间内没有点，设置为0
            averaged_distances.append(avg_distance)

        # 发布结果
        distance_msg = Float32MultiArray()
        distance_msg.data = averaged_distances
        self.publisher_.publish(distance_msg)

        self.get_logger().info('Averaged distances: %s' % str(averaged_distances[10]))

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudDistance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()