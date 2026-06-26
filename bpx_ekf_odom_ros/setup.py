import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'bpx_ekf_odom_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mai',
    maintainer_email='maiyunxi173@gmail.com',
    description='BPX 四足机器人 EKF 实时里程计 ROS2 节点（IMU + 腿式里程计融合）',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ekf_odom_node = bpx_ekf_odom_ros.ekf_odom_node:main',
            'csv_player = bpx_ekf_odom_ros.csv_player:main',
            'odom_recorder = bpx_ekf_odom_ros.odom_recorder:main',
            'walk_measure = bpx_ekf_odom_ros.walk_measure:main',
        ],
    },
)