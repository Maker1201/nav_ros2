import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray, Header
import csv
import math
import numpy as np
import sys
import os

class CSVPlayer(Node):
    def __init__(self):
        super().__init__('csv_player')

        # 发布器
        self.imu_pub = self.create_publisher(Imu, '/bpx/imu/data_raw', 10)
        self.joint_pub = self.create_publisher(JointState, '/bpx/joint_states', 10)
        self.torque_pub = self.create_publisher(Float64MultiArray, '/bpx/joint_torques', 10)

        # 参数
        default_csv = '/home/mai/Desktop/bpx_quad_odom_project/record_message2/record_20260623_121431.csv'
        self.declare_parameter('csv_path', default_csv)
        self.declare_parameter('speed', 1.0)   # 回放速度倍率
        self.declare_parameter('loop', False)  # 是否循环回放

        csv_path = self.get_parameter('csv_path').value
        self.speed = self.get_parameter('speed').value
        self.loop = self.get_parameter('loop').value

        # 检查 CSV 文件是否存在
        if not os.path.exists(csv_path):
            self.get_logger().error(f'CSV 文件不存在: {csv_path}')
            raise FileNotFoundError(f'CSV file not found: {csv_path}')

        # 加载 CSV
        self.rows = []
        with open(csv_path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                self.rows.append(row)

        self.get_logger().info(
            f'已加载 {len(self.rows)} 行数据 | '
            f'speed={self.speed}x | '
            f'loop={"是" if self.loop else "否"}'
        )

        # 关节名称（与 ekf_odom_node.py 一致）
        self.joint_names = [
            'fl_hip_roll_joint', 'fl_hip_pitch_joint', 'fl_knee_joint',
            'fr_hip_roll_joint', 'fr_hip_pitch_joint', 'fr_knee_joint',
            'hl_hip_roll_joint', 'hl_hip_pitch_joint', 'hl_knee_joint',
            'hr_hip_roll_joint', 'hr_hip_pitch_joint', 'hr_knee_joint',
        ]

        # 预计算时间戳（秒）
        self.timestamps = np.array([int(row['timestamp_ms']) for row in self.rows]) / 1000.0

        # 状态
        self.idx = 0
        self.t_start = None      # CSV 起始时间
        self.t0 = None           # ROS 起始时间
        self.loop_count = 0

        # 启动定时器（5ms 间隔检查）
        self.timer = self.create_timer(0.005, self.tick)

    def tick(self):
        if self.idx >= len(self.rows):
            if self.loop:
                self.idx = 0
                self.t_start = None
                self.t0 = None
                self.loop_count += 1
                self.get_logger().info(f'循环回放第 {self.loop_count} 轮...')
                return
            else:
                self.get_logger().info('CSV 回放完毕')
                self.timer.cancel()
                return

        row = self.rows[self.idx]
        t_csv = self.timestamps[self.idx]

        if self.t_start is None:
            self.t_start = t_csv
            self.t0 = self.get_clock().now()

        elapsed_ros = (self.get_clock().now() - self.t0).nanoseconds / 1e9
        target_csv_time = self.t_start + elapsed_ros * self.speed

        if t_csv > target_csv_time:
            return

        # 发布 IMU
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        imu_msg.linear_acceleration.x = float(row['imu_acc_x'])
        imu_msg.linear_acceleration.y = float(row['imu_acc_y'])
        imu_msg.linear_acceleration.z = float(row['imu_acc_z'])
        imu_msg.angular_velocity.x = float(row['imu_omega_x'])
        imu_msg.angular_velocity.y = float(row['imu_omega_y'])
        imu_msg.angular_velocity.z = float(row['imu_omega_z'])
        # SDK 的 imu_quat_* 是退化四元数（w≈0，z≈1），不可用。
        # 改用 imu_rpy_* 重新计算正确四元数（Hamilton 约定）。
        roll  = float(row['imu_rpy_roll'])
        pitch = float(row['imu_rpy_pitch'])
        yaw   = float(row['imu_rpy_yaw'])
        cr, sr = math.cos(roll  / 2), math.sin(roll  / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cy, sy = math.cos(yaw   / 2), math.sin(yaw   / 2)
        imu_msg.orientation.w =  cr * cp * cy + sr * sp * sy
        imu_msg.orientation.x =  sr * cp * cy - cr * sp * sy
        imu_msg.orientation.y =  cr * sp * cy + sr * cp * sy
        imu_msg.orientation.z =  cr * cp * sy - sr * sp * cy
        self.imu_pub.publish(imu_msg)

        # 发布关节状态
        joint_msg = JointState()
        joint_msg.header.stamp = imu_msg.header.stamp
        joint_msg.name = self.joint_names
        joint_msg.position = [float(row[f'q_{i:02d}']) for i in range(12)]
        joint_msg.velocity = [float(row[f'dq_{i:02d}']) for i in range(12)]
        self.joint_pub.publish(joint_msg)

        # 发布关节力矩
        torque_msg = Float64MultiArray()
        torque_msg.data = [float(row[f'tau_{i:02d}']) for i in range(12)]
        self.torque_pub.publish(torque_msg)

        self.idx += 1


def main(args=None):
    rclpy.init(args=args)
    try:
        node = CSVPlayer()
        rclpy.spin(node)
    except FileNotFoundError as e:
        print(f'错误: {e}')
        sys.exit(1)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
