from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='temperature_publisher',
            executable='temperature_publisher',
            name='temperature_publisher',
            output='screen'
        ),
        Node(
            package='yuntai_control',
            executable='yuntai_controller_node',
            name='yuntai_controller_node',
            output='screen'
        ),
        Node(
            package='yuntai_control',
            executable='temperature_tracking_node_v2',
            name='temperature_tracking_node_v2',
            output='screen'
        ),
        Node(
            package='sbus_control',
            executable='sbus_control_node',
            name='sbus_control_node',
            output='screen'
        ),
        Node(
            package='lidar_distance',
            executable='lidar_distance',
            name='lidar_distance',
            output='screen'
        ),
        ############
        # Node(
        #     package='auto_drive',
        #     executable='pointcloud_to_2d_map',
        #     name='pointcloud_to_2d_map',
        #     output='screen'
        # ),
        # Node(
        #     package='auto_drive',
        #     executable='navigation_controller',
        #     name='navigation_controller',
        #     output='screen'
        # ),
        # Node(
        #     package='move_pos',
        #     executable='move_pos_topic_node_v3',
        #     name='move_pos_topic_node_v3',
        #     output='screen'
        # ),
    ])
