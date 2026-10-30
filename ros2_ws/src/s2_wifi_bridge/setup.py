from setuptools import setup
import os
from glob import glob

package_name = 's2_wifi_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@todo.todo',
    description='ROS 2 Wi-Fi bridge for YDLIDAR S2 on ESP32',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = s2_wifi_bridge.bridge_node:main',
            'odom_to_tf_node = s2_wifi_bridge.odom_to_tf_node:main',
            'stationary_amcl_update_node = s2_wifi_bridge.stationary_amcl_update_node:main',
            'lidar_deskew_node = s2_wifi_bridge.lidar_deskew_node:main'
        ],
    },
)
