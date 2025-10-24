from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'auto_drive'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bestway',
    maintainer_email='19858193124@163.com',
    description='Auto drive package with online SLAM navigation',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'auto_drive_node = auto_drive.auto_drive_node:main',
            'pose_printer_node = auto_drive.pose_printer:main',
            'laser_map_builder_node = auto_drive.laser_map_builder:main',
            'search_fire = auto_drive.search_fire:main',
            # 优化的导航控制器
            'navigation_controller = auto_drive.navigation_controller:main',
            'pointcloud_to_2d_map = auto_drive.pointcloud_to_2d_map:main',
            'simple_map_converter = auto_drive.simple_map_converter:main',
            # Nav2集成导航节点
            'navigation_node = auto_drive.navigation_node:main',
            'odom_to_baselink_publisher = auto_drive.odom_to_baselink_publisher:main',
        ],
    },
)
