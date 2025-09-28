from setuptools import find_packages, setup

package_name = 'sbus_control'

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
            'sbus_control_node = sbus_control.sbus_control_node:main',
            'keyboard_control_node = sbus_control.keyboard_control_node:main',
            'serial_node = sbus_control.serial_node:main',
            'ws2812_led_node = sbus_control.ws2812_led_node:main',
        ],
    },
)
