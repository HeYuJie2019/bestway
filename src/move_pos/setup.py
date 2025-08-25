from setuptools import find_packages, setup

package_name = 'move_pos'

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
            'move_pos_topic_node = move_pos.move_pos:main',
            'move_pos_topic_node_v2 = move_pos.move_pos_v2:main',
            'move_pos_topic_node_v3 = move_pos.move_pos_v3:main'
        ],
    },
)
