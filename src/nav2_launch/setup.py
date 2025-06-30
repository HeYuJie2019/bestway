from setuptools import find_packages, setup

package_name = 'nav2_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装launch目录下所有launch文件
        ('share/' + package_name + '/launch', ['launch/nav2_bringup.launch.py']),
        ('share/' + package_name, ['nav2_params.yaml']),  # 安装参数文件到share目录
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bestway',
    maintainer_email='19858193124@163.com',
    description='Nav2导航自定义启动包',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
