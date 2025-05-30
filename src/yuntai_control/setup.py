from setuptools import find_packages, setup

package_name = 'yuntai_control'

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
    maintainer_email='bestway@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yuntai_controller_node = yuntai_control.yuntai_control_node:main',
            'temperature_tracking_node = yuntai_control.temperature_tracking_node:main',
        ],
    },
)
