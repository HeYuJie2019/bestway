# 在v2基础上加入了里程计异常监控功能
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
import math
import time

class GoToPoseTopicNode(Node):
    def __init__(self):
        super().__init__('move_pos_topic_node_v3')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/Odometry', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(Point, '/temp_goal_pose', self.goal_callback, 10)
        # self.goal_sub = self.create_subscription(Point, '/goal_pose', self.goal_callback, 10)
        self.lidar_subscription = self.create_subscription(
            Float32MultiArray,
            'horizontal_distances',
            self.lidar_callback,
            10
        )
        self.current_pose = None
        self.target_point = None
        self.has_goal = False
        self.front_distance = 0.0
        self.left_distance = 0.0
        self.right_distance = 0.0
        self.latest_distances = None
        self.current_speed = 0.0
        self.avoid_obstacle = False
        self.avoid_obstacle_count = 0
        self.avoid_turn_direction = None
        
        # 新增：左右侧避障相关变量
        self.side_avoid_active = False       # 是否正在进行左右侧避障
        self.side_avoid_direction = None     # 左右侧避障方向 ("right" 或 "left")
        self.side_avoid_threshold = 0.7      # 左右侧距离阈值
        
        # 新增：里程计异常监控相关变量
        self.odom_error_count = 0            # 里程计错误计数
        self.max_odom_error_count = 10       # 最大允许的连续错误次数
        self.last_valid_pose = None          # 上一次有效的位姿
        self.odom_timeout_threshold = 2.0    # 里程计超时阈值(秒)
        self.last_odom_time = None           # 上次接收到里程计的时间
        self.position_jump_threshold = 2.0   # 位置跳跃阈值(米)
        self.max_reasonable_speed = 10.0     # 最大合理速度(m/s)
        self.node_should_stop = False        # 节点是否应该停止

    def odom_callback(self, msg):
        current_time = self.get_clock().now()
        self.last_odom_time = current_time
        
        # 检查里程计数据的有效性
        if self.is_odom_data_valid(msg):
            self.current_pose = msg.pose.pose
            self.last_valid_pose = msg.pose.pose
            self.odom_error_count = 0  # 重置错误计数
            self.get_logger().debug("里程计数据正常")
        else:
            self.odom_error_count += 1
            self.get_logger().warn(f"里程计数据异常！连续错误次数: {self.odom_error_count}/{self.max_odom_error_count}")
            
            # 如果连续错误次数超过阈值，标记节点应该停止
            if self.odom_error_count >= self.max_odom_error_count:
                self.get_logger().error("里程计数据持续异常，节点将停止运行！")
                self.node_should_stop = True
                # 发送停止命令
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
    
    def is_odom_data_valid(self, msg):
        """检查里程计数据是否有效"""
        try:
            pose = msg.pose.pose
            position = pose.position
            orientation = pose.orientation
            
            # 检查1: 位置和姿态是否包含NaN或无穷大
            if (math.isnan(position.x) or math.isnan(position.y) or math.isnan(position.z) or
                math.isinf(position.x) or math.isinf(position.y) or math.isinf(position.z)):
                self.get_logger().warn("里程计位置包含NaN或无穷大值")
                return False
                
            if (math.isnan(orientation.x) or math.isnan(orientation.y) or 
                math.isnan(orientation.z) or math.isnan(orientation.w) or
                math.isinf(orientation.x) or math.isinf(orientation.y) or 
                math.isinf(orientation.z) or math.isinf(orientation.w)):
                self.get_logger().warn("里程计姿态包含NaN或无穷大值")
                return False
            
            # 检查2: 四元数是否归一化
            quat_norm = math.sqrt(orientation.x**2 + orientation.y**2 + 
                                orientation.z**2 + orientation.w**2)
            if abs(quat_norm - 1.0) > 0.1:  # 允许一定的误差
                self.get_logger().warn(f"四元数未归一化，模长: {quat_norm}")
                return False
            
            # 检查3: 位置是否在合理范围内
            max_position = 1000.0  # 假设机器人不会超出1000米的范围
            if (abs(position.x) > max_position or abs(position.y) > max_position or 
                abs(position.z) > max_position):
                self.get_logger().warn(f"位置超出合理范围: ({position.x}, {position.y}, {position.z})")
                return False
            
            # 检查4: 位置跳跃检测
            if self.last_valid_pose is not None:
                last_pos = self.last_valid_pose.position
                dx = position.x - last_pos.x
                dy = position.y - last_pos.y
                dz = position.z - last_pos.z
                distance_jump = math.sqrt(dx**2 + dy**2 + dz**2)
                
                # 估算时间间隔（假设里程计频率约为10-50Hz）
                estimated_dt = 0.1  # 估算的时间间隔
                estimated_speed = distance_jump / estimated_dt
                
                if distance_jump > self.position_jump_threshold:
                    self.get_logger().warn(f"检测到位置跳跃: {distance_jump:.2f}m，估算速度: {estimated_speed:.2f}m/s")
                    if estimated_speed > self.max_reasonable_speed:
                        return False
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"里程计数据验证时发生异常: {str(e)}")
            return False

    def goal_callback(self, msg):
        self.target_point = msg
        self.has_goal = True

    def lidar_callback(self, msg):
        self.latest_distances = msg.data
        num_points = len(self.latest_distances)
        if num_points != 21:
            self.get_logger().warn("接收到的水平距离数据数量与预期不符！")
            return
        # 0~20: 从右到左正前方180度
        # 8~12为正前方，13~20为左，0~7为右
        front_distances = self.latest_distances[8:13]
        left_distances = self.latest_distances[13:]
        right_distances = self.latest_distances[:8]
        self.front_distance = min(front_distances)
        self.left_distance = min(left_distances)
        self.right_distance = min(right_distances)
        
        # 检查左右侧距离是否需要避障
        self.check_side_obstacles()

    def get_front_distance(self):
        # 直接返回激光雷达正前方最小距离
        return self.front_distance
    
    def check_side_obstacles(self):
        """检查左右侧是否有障碍物需要避让"""
        if self.latest_distances is None or len(self.latest_distances) != 21:
            return
            
        # 检查左侧距离
        left_min = self.left_distance
        # 检查右侧距离  
        right_min = self.right_distance
        
        # 判断是否需要进行左右侧避障
        if left_min < self.side_avoid_threshold:
            # 左侧太近，需要向右偏
            self.side_avoid_active = True
            self.side_avoid_direction = "right"
            self.get_logger().info(f"左侧距离过近({left_min:.2f}m < {self.side_avoid_threshold}m)，向右偏移")
        elif right_min < self.side_avoid_threshold:
            # 右侧太近，需要向左偏
            self.side_avoid_active = True
            self.side_avoid_direction = "left"
            self.get_logger().info(f"右侧距离过近({right_min:.2f}m < {self.side_avoid_threshold}m)，向左偏移")
        else:
            # 左右侧距离都足够，取消侧避障
            if self.side_avoid_active:
                self.get_logger().info(f"左右侧距离恢复正常(左:{left_min:.2f}m, 右:{right_min:.2f}m)，取消侧避障")
            self.side_avoid_active = False
            self.side_avoid_direction = None

    def check_odom_timeout(self):
        """检查里程计是否超时"""
        if self.last_odom_time is None:
            return False
            
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_odom_time).nanoseconds / 1e9
        
        if time_diff > self.odom_timeout_threshold:
            self.get_logger().warn(f"里程计数据超时: {time_diff:.2f}s > {self.odom_timeout_threshold}s")
            return True
        return False

    def control_loop(self):
        # PID参数
        linear_kp = 1.8
        linear_ki = 0.0
        linear_kd = 0.5
        angular_kp = 5.0
        angular_ki = 0.0
        angular_kd = 0.6

        prev_linear_error = 0.0
        sum_linear_error = 0.0
        prev_angular_error = 0.0
        sum_angular_error = 0.0

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            
            # 检查节点是否应该停止
            if self.node_should_stop:
                self.get_logger().error("由于里程计数据异常，节点已停止运行")
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                break
            
            # 检查里程计超时
            if self.check_odom_timeout():
                self.get_logger().error("里程计数据超时，节点停止运行")
                self.node_should_stop = True
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                continue
            
            if not self.has_goal or self.target_point is None:
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                continue
            # 检查目标点有效性
            if math.isnan(self.target_point.x) or math.isnan(self.target_point.y):
                self.has_goal = False
                self.target_point = None
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                continue
            if self.current_pose is None:
                self.get_logger().warn("当前位姿未更新，等待中...")
                # 检查是否是里程计问题导致的
                if self.last_odom_time is not None:
                    current_time = self.get_clock().now()
                    time_diff = (current_time - self.last_odom_time).nanoseconds / 1e9
                    if time_diff > self.odom_timeout_threshold:
                        self.get_logger().error("里程计长时间未更新位姿信息")
                        self.node_should_stop = True
                continue
            dx = self.target_point.x - self.current_pose.position.x
            dy = self.target_point.y - self.current_pose.position.y
            distance = math.hypot(dx, dy)
            angle_to_goal = math.atan2(dy, dx)
            q = self.current_pose.orientation
            yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0-2.0*(q.y*q.y + q.z*q.z))
            angle_error = angle_to_goal - yaw
            angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi
            front_distance = self.get_front_distance()
            base_safe_distance = 1.2
            max_safe_distance = 3.0
            speed_factor = abs(self.current_speed) / 20.0
            safe_distance = base_safe_distance + speed_factor * (max_safe_distance - base_safe_distance)
            safe_distance = max(0.5, min(safe_distance, max_safe_distance))
            # --- 智能绕障：主目标方向畅通性实时判断 ---
            if self.avoid_obstacle and front_distance < safe_distance and self.avoid_turn_direction is None:
                left_distance = getattr(self, 'left_distance', 1.0)
                right_distance = getattr(self, 'right_distance', 1.0)
                prefer_direction = "left" if angle_error > 0 else "right"
                if left_distance < 0.8 or right_distance < 0.8:
                    turn_direction = "left" if left_distance > right_distance else "right"
                else:
                    turn_direction = prefer_direction
                self.avoid_turn_direction = turn_direction
            elif front_distance > safe_distance:
                self.avoid_obstacle = False
                self.avoid_turn_direction = None
                self.avoid_straight_count = 0
            
            # 检查是否需要避障（前方或左右侧）
            need_avoid = (self.avoid_obstacle or self.side_avoid_active) and self.avoid_obstacle
            
            if need_avoid:
                twist = Twist()
                twist.linear.x = 0.0
                
                # 确定避障方向
                if self.avoid_obstacle:
                    # 前方避障
                    turn_direction = getattr(self, 'avoid_turn_direction', 'left')
                    twist.angular.z = 4.0 if turn_direction == "left" else -4.0
                    self.get_logger().info(
                        f"前方避障中，方向: {turn_direction}，前方距离: {front_distance:.2f}，安全距离: {safe_distance:.2f}"
                    )
                elif self.side_avoid_active:
                    # 左右侧避障
                    side_avoid_intensity = 3.0
                    if self.side_avoid_direction == "left":
                        twist.angular.z = side_avoid_intensity
                    elif self.side_avoid_direction == "right":
                        twist.angular.z = -side_avoid_intensity
                    self.get_logger().info(
                        f"侧避障中，方向: {self.side_avoid_direction}，左距离: {self.left_distance:.2f}，右距离: {self.right_distance:.2f}"
                    )
                
                self.cmd_vel_pub.publish(twist)
                continue
            # 到达目标
            if distance < 0.5:
                twist = Twist()
                self.cmd_vel_pub.publish(twist)
                self.get_logger().info("到达目标点，等待新目标...")
                self.get_logger().info(
                    f"当前位置: ({self.current_pose.position.x:.3f},{self.current_pose.position.y:.3f}), "
                    f"到达目标点，等待新目标..."
                )
                self.has_goal = False
                continue
            # PID控制
            linear_error = distance
            sum_linear_error += linear_error
            d_linear_error = linear_error - prev_linear_error
            linear_speed = linear_kp * linear_error + linear_ki * sum_linear_error + linear_kd * d_linear_error
            prev_linear_error = linear_error
            angular_error = angle_error
            sum_angular_error += angular_error
            d_angular_error = angular_error - prev_angular_error
            angular_speed = angular_kp * angular_error + angular_ki * sum_angular_error + angular_kd * d_angular_error
            prev_angular_error = angular_error
            linear_speed = max(min(linear_speed, 4.0), -4.0)
            self.current_speed = linear_speed
            angular_speed = max(min(angular_speed, 6.0), -6.0)
            
            twist = Twist()
            if abs(angle_error) > 0.45:
                self.avoid_obstacle = False
                twist.linear.x = 0.0
                twist.angular.z = angular_speed
            else:
                self.avoid_obstacle = True
                twist.linear.x = linear_speed
                twist.angular.z = 0.0
            
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info(
                f"当前位置: ({self.current_pose.position.x:.3f},{self.current_pose.position.y:.3f}), "
                f"目标:({self.target_point.x:.3f},{self.target_point.y:.3f}), "
                f"速度: {twist.linear.x:.2f}, 角速度: {twist.angular.z:.2f}, "
                f"前方距离: {front_distance:.2f}, 安全距离: {safe_distance:.2f}, "
                f"左距离: {self.left_distance:.2f}, 右距离: {self.right_distance:.2f}, "
                f"侧避障: {'是' if self.side_avoid_active else '否'}({self.side_avoid_direction or '无'}), "
                f"里程计错误: {self.odom_error_count}/{self.max_odom_error_count}"
            )

def main(args=None):
    rclpy.init(args=args)
    node = GoToPoseTopicNode()
    try:
        node.control_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()