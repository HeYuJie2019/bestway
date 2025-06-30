# filepath: /home/bestway/bestway_ws/src/nav2_launch/launch/nav2_bringup.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    nav2_params = os.path.join(
        get_package_share_directory('nav2_launch'),
        'nav2_params.yaml'
    )
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={'params_file': nav2_params}.items()
        ),
    ])