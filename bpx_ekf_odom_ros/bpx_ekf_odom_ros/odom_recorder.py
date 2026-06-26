#!/usr/bin/env python3
"""
odom_recorder.py - 订阅 /bpx/odom，将轨迹写入 CSV 文件

输出列（与 run_odometry.py 的 _record() 对齐）：
  t_ros, px, py, pz, vx, vy, vz, roll, pitch, yaw, qw, qx, qy, qz

用法：
  ros2 run bpx_ekf_odom_ros odom_recorder \\
    --ros-args -p out_csv:=/home/mai/online_odom_clean.csv
"""

import math
import csv
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry


def quat_to_rpy(w, x, y, z):
    """四元数 → roll/pitch/yaw（弧度）。"""
    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class OdomRecorder(Node):
    def __init__(self):
        super().__init__('odom_recorder')

        self.declare_parameter('out_csv', '/home/mai/online_odom_clean.csv')
        out_csv = self.get_parameter('out_csv').value

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        self._sub = self.create_subscription(
            Odometry, '/bpx/odom', self._callback, sensor_qos)

        self._f = open(out_csv, 'w', newline='')
        self._writer = csv.writer(self._f)
        self._writer.writerow([
            't_ros',
            'px', 'py', 'pz',
            'vx', 'vy', 'vz',
            'roll', 'pitch', 'yaw',
            'qw', 'qx', 'qy', 'qz',
        ])
        self._count = 0

        self.get_logger().info(f'odom_recorder 启动 → {out_csv}')

    def _callback(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        q = msg.pose.pose.orientation
        roll, pitch, yaw = quat_to_rpy(q.w, q.x, q.y, q.z)

        self._writer.writerow([
            f'{t:.6f}',
            f'{p.x:.6f}', f'{p.y:.6f}', f'{p.z:.6f}',
            f'{v.x:.6f}', f'{v.y:.6f}', f'{v.z:.6f}',
            f'{roll:.6f}', f'{pitch:.6f}', f'{yaw:.6f}',
            f'{q.w:.6f}', f'{q.x:.6f}', f'{q.y:.6f}', f'{q.z:.6f}',
        ])
        self._count += 1
        if self._count % 500 == 0:
            self._f.flush()
            self.get_logger().info(f'已录制 {self._count} 帧')

    def destroy_node(self):
        self._f.flush()
        self._f.close()
        self.get_logger().info(f'录制完成，共 {self._count} 帧')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OdomRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
