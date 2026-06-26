#!/usr/bin/env python3
"""
ekf_odom.launch.py - 启动 EKF 实时里程计节点

上真机用法（与 bpx_driver 配合）：
  # 终端1：先启动驱动（禁用驱动自带 TF，避免与 EKF 冲突）
  ros2 launch bpx_driver bpx_driver.launch.py publish_odom_tf:=false

  # 终端2：启动 EKF
  ros2 launch bpx_ekf_odom_ros ekf_odom.launch.py

话题重映射（自动）：
  EKF 内部订阅名        →  bpx_driver 实际发布名
  /bpx/imu/data_raw    →  /imu/data
  /bpx/joint_states    →  /joint_states

参数（可通过命令行覆盖）：
  odom_frame      里程计坐标系（默认 odom）
  base_frame      机身坐标系（默认 base_link）
  publish_tf      是否发布 tf（默认 true）
  publish_rate    发布频率上限 Hz（默认 100.0）
  wsum_min        速度更新最小权重和（默认 0.05）
  height_lock     竖直弱约束开关（默认 true）
  R_pz            竖直观测方差（默认 0.04）
  pz_ref          参考高度（默认 0.0）
  use_sqrt        使用 sqrt 缩放 R（默认 true）
  chi2_soft       innovation 软门阈值（默认 50.0）
  repeat_R_factor 重复帧 R 放大倍数（默认 4.0）
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bpx_ekf_odom_ros',
            executable='ekf_odom_node',
            name='ekf_odom_node',
            output='screen',
            parameters=[{
                'odom_frame': LaunchConfiguration('odom_frame', default='odom'),
                'base_frame': LaunchConfiguration('base_frame', default='base_link'),
                'publish_tf': LaunchConfiguration('publish_tf', default='true'),
                'publish_rate': LaunchConfiguration('publish_rate', default='100.0'),
                'wsum_min': LaunchConfiguration('wsum_min', default='0.05'),
                'height_lock': LaunchConfiguration('height_lock', default='true'),
                'R_pz': LaunchConfiguration('R_pz', default='0.04'),
                'pz_ref': LaunchConfiguration('pz_ref', default='0.0'),
                'use_sqrt': LaunchConfiguration('use_sqrt', default='true'),
                'chi2_soft': LaunchConfiguration('chi2_soft', default='50.0'),
                'repeat_R_factor': LaunchConfiguration('repeat_R_factor', default='4.0'),
            }],
            # 将 EKF 内部话题名重映射到 bpx_driver 实际发布的话题名
            # 格式：(EKF内部订阅名, bpx_driver实际发布名)
            remappings=[
                ('/bpx/imu/data_raw', '/imu/data'),
                ('/bpx/joint_states', '/joint_states'),
            ],
        ),
    ])