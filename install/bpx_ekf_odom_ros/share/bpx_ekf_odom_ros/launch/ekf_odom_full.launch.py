#!/usr/bin/env python3
"""
ekf_odom_full.launch.py - 一键启动完整验证流水线

节点：
  1. ekf_odom_node  — 实时 EKF 里程计（订阅传感器话题，发布 /bpx/odom + tf）
  2. csv_player     — CSV 数据回放（模拟传感器）
  3. odom_recorder  — 录制 /bpx/odom 到 CSV（对比用）

用法：
  ros2 launch bpx_ekf_odom_ros ekf_odom_full.launch.py

可选参数（命令行覆盖）：
  csv_path      输入 CSV 路径（默认内置路径）
  speed         回放速度倍率（默认 1.0）
  loop          是否循环回放（默认 false）
  out_csv       录制输出 CSV 路径（默认 /home/mai/online_odom_clean.csv）
  odom_frame    里程计坐标系（默认 odom）
  base_frame    机身坐标系（默认 base_link）
  publish_tf    是否发布 tf（默认 true）
  height_lock   竖直弱约束（默认 true）

Ctrl+C 结束后：
  - online_odom_clean.csv 即为在线轨迹
  - 运行对比脚本：
      python3 -m bpx_ekf_odom_ros.compare_odom
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

_DEFAULT_CSV = (
    '/home/mai/Desktop/bpx_quad_odom_project/'
    'record_message2/record_20260623_121431.csv'
)
_DEFAULT_OUT = '/home/mai/online_odom_clean.csv'


def generate_launch_description():
    return LaunchDescription([
        # ── EKF 里程计节点 ───────────────────────────────────────────
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
        ),

        # ── CSV 数据回放节点 ─────────────────────────────────────────
        Node(
            package='bpx_ekf_odom_ros',
            executable='csv_player',
            name='csv_player',
            output='screen',
            parameters=[{
                'csv_path': LaunchConfiguration('csv_path', default=_DEFAULT_CSV),
                'speed': LaunchConfiguration('speed', default='1.0'),
                'loop': LaunchConfiguration('loop', default='false'),
            }],
        ),

        # ── /bpx/odom 录制节点 ───────────────────────────────────────
        Node(
            package='bpx_ekf_odom_ros',
            executable='odom_recorder',
            name='odom_recorder',
            output='screen',
            parameters=[{
                'out_csv': LaunchConfiguration('out_csv', default=_DEFAULT_OUT),
            }],
        ),
    ])
