#!/usr/bin/env python3
"""
walk_measure.py - 行走测试里程计精度测量节点

订阅 /bpx/odom，从第一帧开始计时，Ctrl+C 时打印测量摘要。

用法：
  ros2 run bpx_ekf_odom_ros walk_measure

参数：
  odom_topic   订阅话题（默认 /bpx/odom）
  save_csv     结果保存路径（默认空，不保存）

服务：
  ~/reset      std_srvs/srv/Trigger — 重置起始点，开始新一轮测量
"""

import math
import csv
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger


class WalkMeasure(Node):
    def __init__(self):
        super().__init__('walk_measure')

        self.declare_parameter('odom_topic', '/bpx/odom')
        self.declare_parameter('save_csv', '')

        topic = self.get_parameter('odom_topic').value
        self._save_csv = self.get_parameter('save_csv').value

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        self._sub = self.create_subscription(Odometry, topic, self._cb, sensor_qos)
        self._reset_srv = self.create_service(Trigger, '~/reset', self._reset_cb)

        self._reset()
        self.get_logger().info(f'walk_measure 启动，订阅 {topic}')
        self.get_logger().info('等待第一帧数据，机器人站稳后开始走…')
        self.get_logger().info('重置起点：ros2 service call /walk_measure/reset std_srvs/srv/Trigger')

    # ── 内部状态 ──────────────────────────────────────────────────────

    def _reset(self):
        self._start_pos = None
        self._start_t = None
        self._samples = []  # (elapsed, dist, dx, dy, dz)

    def _reset_cb(self, _req, resp):
        self._reset()
        resp.success = True
        resp.message = '起始点已重置'
        self.get_logger().info('── 重置起始点 ──')
        return resp

    # ── 数据回调 ──────────────────────────────────────────────────────

    def _cb(self, msg: Odometry):
        p = msg.pose.pose.position
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self._start_pos is None:
            self._start_pos = (p.x, p.y, p.z)
            self._start_t = t
            self.get_logger().info(
                f'开始测量  起点=({p.x:.3f}, {p.y:.3f}, {p.z:.3f})'
            )
            return

        elapsed = t - self._start_t
        dx = p.x - self._start_pos[0]
        dy = p.y - self._start_pos[1]
        dz = p.z - self._start_pos[2]
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        self._samples.append((elapsed, dist, dx, dy, dz))

        if elapsed > 0 and len(self._samples) % 50 == 0:
            v = msg.twist.twist.linear
            speed = math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)
            self.get_logger().info(
                f't={elapsed:.1f}s  dist={dist:.2f}m  |v|={speed:.2f}m/s  '
                f'dx={dx:.2f} dy={dy:.2f} dz={dz:.2f}'
            )

    # ── 结束时摘要 ────────────────────────────────────────────────────

    def _print_summary(self):
        if not self._samples:
            self.get_logger().warn('无测量数据')
            return

        elapsed, dist, dx, dy, dz = self._samples[-1]
        avg_speed = dist / elapsed if elapsed > 0 else 0.0
        linearity = dx / dist * 100.0 if dist > 0 else 0.0

        lines = [
            '',
            '========== 行走测试结果 ==========',
            f'总时间    : {elapsed:.1f} s',
            f'总距离    : {dist:.3f} m',
            f'平均速度  : {avg_speed:.2f} m/s',
            f'前进 dx   : {dx:.3f} m',
            f'侧移 dy   : {dy:.3f} m  (目标 <0.2)',
            f'高度变化 dz: {dz:.3f} m  (目标 <0.1)',
            f'直线度    : {linearity:.1f}%  (目标 >80%)',
            '==================================',
        ]
        for line in lines:
            self.get_logger().info(line)

        if self._save_csv:
            self._write_csv()

    def _write_csv(self):
        path = self._save_csv
        os.makedirs(os.path.dirname(path) if os.path.dirname(path) else '.', exist_ok=True)
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['elapsed_s', 'dist_m', 'dx_m', 'dy_m', 'dz_m'])
            w.writerows(self._samples)
        self.get_logger().info(f'轨迹数据已保存 → {path}')

    def destroy_node(self):
        self._print_summary()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WalkMeasure()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
