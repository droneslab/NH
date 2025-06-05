import os
from glob import glob
from setuptools import setup

package_name = 'nighthawk_ros'

setup(
    name=package_name,
    version='0.0.1',  # bumped version for lifecycle updates
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        # Only pick up Python launch files (ending in .launch.py)
        (os.path.join('share', package_name, 'launch'), glob('launch/*_launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='ROS2 nodes with lifecycle management for Nighthawk robotics, including LED control.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nighthawk_ros = nighthawk_ros.nighthawk_ros:main',
            'light_control_ros = nighthawk_ros.light_control_ros:main',
            'nighthawk_score_node = nighthawk_ros.nighthawk_score_node:main',
            'image_writer = nighthawk_ros.image_writer:main',
            'image_metrics_node = nighthawk_ros.metric_publisher:main' ,
            'image_saver = nighthawk_ros.image_saver:main',
            'lux_reader = nighthawk_ros.lux_reader:main'
        ],
    },
)