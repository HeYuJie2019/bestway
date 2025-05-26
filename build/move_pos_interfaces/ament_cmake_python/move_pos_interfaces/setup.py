from setuptools import find_packages
from setuptools import setup

setup(
    name='move_pos_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('move_pos_interfaces', 'move_pos_interfaces.*')),
)
