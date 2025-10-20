import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    # SDK 动态库路径，确保节点运行时可找到 libSV_NET_SDK.so
    sdk_lib_path = "/home/bestway/SV_NET_SDK_ARM_250424/demo/consoleDemo/lib/x64/"
    merged_ld_path = os.environ.get("LD_LIBRARY_PATH", "")
    if sdk_lib_path not in merged_ld_path.split(":"):
        merged_ld_path = f"{merged_ld_path}:{sdk_lib_path}" if merged_ld_path else sdk_lib_path

    return LaunchDescription([
        # Node(
        #     package='yuntai_control',
        #     executable='yuntai_controller_node',
        #     name='yuntai_controller_node',
        #     output='screen'
        # ),
        # Node(
        #     package='yuntai_control',
        #     executable='temperature_tracking_node_v2',
        #     name='temperature_tracking_node_v2',
        #     output='screen'
        # ),
        Node(
            package='sbus_control',
            executable='sbus_control_node',
            name='sbus_control_node',
            output='screen'
        ),
        # Node(
        #     package='sbus_control',
        #     executable='serial_node',
        #     name='serial_node',
        #     output='screen'
        # ),
        Node(
            package='lidar_distance',
            executable='lidar_distance',
            name='lidar_distance',
            output='screen'
        ),
        # temperature_publisher 放在最后，并延时启动，避免依赖未就绪
        # Node(
        #     package='temperature_publisher',
        #     executable='temperature_publisher',
        #     name='temperature_publisher',
        #     output='screen'
        # ),
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
