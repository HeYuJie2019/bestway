#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    # 包路径
    pkg_share = FindPackageShare('auto_drive')
    
    return LaunchDescription([
        # 声明启动参数
        # DeclareLaunchArgument(
        #     'input_topic',
        #     default_value='/cloud_registered',
        #     default_value='/Laser_map',
        #     description='Input pointcloud topic from fast_lio2'
        # ),
        
        DeclareLaunchArgument(
            'map_topic', 
            default_value='/map',
            description='Output occupancy grid topic for navigation'
        ),
        
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Whether to launch rviz2'
        ),
        
        DeclareLaunchArgument(
            'use_navigation',
            default_value='true',
            description='Whether to launch navigation controller'
        ),
        
        # 点云到二维地图转换节点
        Node(
            package='auto_drive',
            executable='pointcloud_to_2d_map',
            name='pointcloud_to_2d_map',
            output='screen',
            # parameters=[{
            #     'input_topic': LaunchConfiguration('input_topic'),
            #     'output_topic': LaunchConfiguration('map_topic'),
            #     'map_frame': 'map',
            #     'map_resolution': 0.05,
            #     'map_width': 2000,
            #     'map_height': 2000,
            #     'obstacle_z_min': -0.3,
            #     'obstacle_z_max': 1.0,
            #     # 'update_rate': 5.0,            # 进一步降低更新频率
            #     'update_rate': 50.0,
            #     'inflation_radius': 0.15,
            #     'fast_mode': True,
            #     'skip_frames': 20,             # 进一步增加跳帧
            #     'max_points_per_frame': 10000  # 限制每帧处理的点数
            # }]
        ),
        
        # 导航控制器
        Node(
            package='auto_drive',
            executable='navigation_controller',
            name='navigation_controller',
            output='screen',
            parameters=[{
                'map_topic': LaunchConfiguration('map_topic'),
                'odometry_topic': '/Odometry',           # 使用里程计获取自身位姿
                'goal_topic': '/goal_pose',
                'temp_goal_topic': '/temp_goal_pose',
                'path_topic': '/temp_path',
                'map_frame': 'map',
                'waypoint_min_spacing': 0.5,
                'obstacle_clearance': 0.3,
                'control_rate': 10.0,
                'final_goal_tolerance': 0.3,
            }],
            condition=IfCondition(LaunchConfiguration('use_navigation'))
        ),
        
        # 静态TF发布器 (map -> camera_init)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_camera_init_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_init'],
            output='screen'
        ),
        
        # # 启动rviz2
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     arguments=['-d', PathJoinSubstitution([
        #         pkg_share, 'rviz', 'lightweight_mapping.rviz'
        #     ])],
        #     condition=IfCondition(LaunchConfiguration('use_rviz'))
        # ),
    ])
