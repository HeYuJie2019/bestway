from setuptools import find_packages, setup

package_name = 'auto_drive'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bestway',
    maintainer_email='19858193124@163.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'auto_drive_node = auto_drive.auto_drive_node:main',
            'search_fire_node = auto_drive.search_fire:main',
            'pose_printer_node = auto_drive.pose_printer:main',
            'pointcloud_to_2d_map = auto_drive.pointcloud_to_2d_map:main',
            'navigation_controller = auto_drive.navigation_controller:main',
        ],
    },
)
