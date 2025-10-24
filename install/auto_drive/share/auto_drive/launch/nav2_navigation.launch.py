#!/usr/bin/env python3
"""
完整的Nav2导航系统启动文件
包含：点云建图 + Nav2路径规划 + 临时目标点发布
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # 包路径
    pkg_share = FindPackageShare('auto_drive')
    nav2_bringup_dir = FindPackageShare('nav2_bringup')
    
    # 配置文件路径
    nav2_params_file = PathJoinSubstitution([pkg_share, 'config', 'nav2_params.yaml'])
    
    return LaunchDescription([
        
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Whether to launch rviz2'
        ),
        
        DeclareLaunchArgument(
            'use_nav2',
            default_value='true',
            description='Whether to launch Nav2 stack'
        ),
        
        DeclareLaunchArgument(
            'params_file',
            default_value=nav2_params_file,
            description='Full path to the Nav2 parameters file'
        ),
        
        # 点云到二维地图转换节点
        Node(
            package='auto_drive',
            executable='pointcloud_to_2d_map',
            name='pointcloud_to_2d_map',
            output='screen',
            parameters=[{
                # 'update_rate': 5.0,  # 降低更新频率，Nav2不需要太频繁
                'inflation_radius': 0.25,  # 从0.15增大到0.25，让障碍物更明显
            }]
        ),
        
        # Nav2 Planner Server（仅路径规划）
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
            condition=IfCondition(LaunchConfiguration('use_nav2'))
        ),
        
        # Nav2 Lifecycle Manager（管理规划器生命周期）
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['planner_server']
            }],
            condition=IfCondition(LaunchConfiguration('use_nav2'))
        ),
        
        # TF发布器：从里程计发布odom->base_link
        Node(
            package='auto_drive',
            executable='odom_to_baselink_publisher',
            name='odom_to_baselink_publisher',
            output='screen'
        ),
        
        # 静态TF发布器 (map -> odom)
        # FAST-LIO的里程计通常在map坐标系下，所以map和odom重合
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen'
        ),
        
        # 静态TF发布器 (base_link -> body)
        # body是FAST-LIO的机器人坐标系
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_body_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'body'],
            output='screen'
        ),
        
        # 静态TF发布器 (map -> camera_init) - 保持与FAST-LIO兼容
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_camera_init_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_init'],
            output='screen'
        ),
        
        # 我们的Nav2集成导航节点
        Node(
            package='auto_drive',
            executable='navigation_node',
            name='nav2_navigation_node',
            output='screen',
            parameters=[{
                'planner_id': 'GridBased',
                'temp_goal_spacing': 1.0,
                'temp_goal_switch_distance': 0.6,
                'goal_reached_distance': 0.3,
                'replan_frequency': 2.0,
            }],
            condition=IfCondition(LaunchConfiguration('use_nav2'))
        ),
        
        # 启动rviz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', PathJoinSubstitution([
                pkg_share, 'rviz', 'lightweight_mapping.rviz'
            ])],
            condition=IfCondition(LaunchConfiguration('use_rviz'))
        ),
    ])
