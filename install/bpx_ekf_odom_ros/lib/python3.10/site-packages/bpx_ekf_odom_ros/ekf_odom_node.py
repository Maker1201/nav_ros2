#!/usr/bin/env python3
"""
ekf_odom_node.py - ROS2 实时 EKF 里程计节点

订阅：
  /bpx/imu/data_raw       sensor_msgs/Imu      —— IMU 数据（加速度、角速度）
  /bpx/joint_states       sensor_msgs/JointState —— 关节角度、角速度
  /bpx/joint_torques      std_msgs/Float64MultiArray —— 关节力矩（可选）

发布：
  /bpx/odom               nav_msgs/Odometry    —— EKF 融合里程计
  /tf                     tf2_msgs/TFMessage   —— odom → base_link 变换

内部流程（每帧）：
  1. IMU 预测步（predict）
  2. 正运动学计算足端位置/速度
  3. 触地检测（足端速度法）
  4. 腿式里程计速度估计（ZUPT）
  5. 速度更新 + 竖直弱约束
  6. 注入 + reset
  7. 发布 Odometry + tf

数据流完全对齐 run_odometry.py 的 B 路逻辑。
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster

# 导入 EKF 核心算法
from .config import (
    build_P0, GRAVITY, R_BASE_VEL,
    JOINT_Q_COLS, JOINT_DQ_COLS, JOINT_TAU_COLS,
    LEG_JOINT_MAP,
)
from .kinematics import compute_all_feet, leg_odometry_velocity
from .contact_detector import (
    detect_all_legs, support_weights_for_odometry, compute_torque_baseline,
)
from .adaptive_obs import compute_velocity_meas_noise
from .ekf_core import (
    predict, update_velocity, update_height, inject_attitude, reset_jacobian,
    R_to_quat, extract_pose_covariance, extract_twist_covariance,
)


# ============================================================
# 关节名称 → 索引映射
# 支持两套命名：
#   URDF 命名（离线 CSV / 仿真）
#   bpx_driver 真机命名（leg{0-3}_{abad/hip/knee}）
#     leg0=FL, leg1=FR, leg2=HL, leg3=HR
#     abad=hip_roll, hip=hip_pitch, knee=knee
# ============================================================
JOINT_NAME_TO_IDX = {
    # URDF 命名
    'fl_hip_roll_joint': 0, 'fl_hip_pitch_joint': 1, 'fl_knee_joint': 2,
    'fr_hip_roll_joint': 3, 'fr_hip_pitch_joint': 4, 'fr_knee_joint': 5,
    'hl_hip_roll_joint': 6, 'hl_hip_pitch_joint': 7, 'hl_knee_joint': 8,
    'hr_hip_roll_joint': 9, 'hr_hip_pitch_joint': 10, 'hr_knee_joint': 11,
    # bpx_driver 真机命名
    'leg0_abad': 0, 'leg0_hip': 1, 'leg0_knee': 2,
    'leg1_abad': 3, 'leg1_hip': 4, 'leg1_knee': 5,
    'leg2_abad': 6, 'leg2_hip': 7, 'leg2_knee': 8,
    'leg3_abad': 9, 'leg3_hip': 10, 'leg3_knee': 11,
}


class EKFOdomNode(Node):
    """
    ROS2 实时 EKF 里程计节点。

    参数（可通过 ROS2 参数服务器动态配置）：
      ~odom_frame         里程计坐标系（默认 "odom"）
      ~base_frame         机身坐标系（默认 "base_link"）
      ~publish_tf         是否发布 tf（默认 True）
      ~publish_rate       发布频率上限（Hz，默认 100）
      ~wsum_min           速度更新最小权重和（默认 0.05）
      ~height_lock        竖直弱约束开关（默认 True）
      ~R_pz               竖直观测方差（默认 0.04）
      ~pz_ref             参考高度（默认 0.0）
      ~use_sqrt           使用 sqrt 缩放 R（默认 True）
      ~chi2_soft          innovation 软门阈值（默认 50.0）
      ~repeat_R_factor    重复帧 R 放大倍数（默认 4.0）
    """

    def __init__(self):
        super().__init__('ekf_odom_node')

        # ============================================================
        # 参数声明
        # ============================================================
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_rate', 100.0)
        self.declare_parameter('wsum_min', 0.05)
        self.declare_parameter('height_lock', True)
        self.declare_parameter('R_pz', 0.04)
        self.declare_parameter('pz_ref', 0.0)
        self.declare_parameter('use_sqrt', True)
        self.declare_parameter('chi2_soft', 50.0)
        self.declare_parameter('repeat_R_factor', 4.0)

        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._publish_tf = self.get_parameter('publish_tf').value
        self._wsum_min = self.get_parameter('wsum_min').value
        self._height_lock = self.get_parameter('height_lock').value
        self._R_pz = self.get_parameter('R_pz').value
        self._pz_ref = self.get_parameter('pz_ref').value
        self._use_sqrt = self.get_parameter('use_sqrt').value
        self._chi2_soft = self.get_parameter('chi2_soft').value
        self._repeat_R_factor = self.get_parameter('repeat_R_factor').value

        # ============================================================
        # EKF 状态
        # ============================================================
        self._x = np.zeros(15)          # 误差状态向量
        self._P = build_P0()            # 协方差矩阵
        self._R_body = np.eye(3)        # 机身姿态（世界系）
        self._initialized = False       # 是否已初始化
        self._gravity = GRAVITY         # 重力大小（初始化时从 IMU 实测更新）

        # 力矩基线（从启动后静态段累积估计）
        self._tau_baseline = np.zeros((4, 3))
        self._tau_static_buf = []           # 缓冲静态帧力矩 (N, 12)
        self._tau_baseline_ready = False
        # 前 N_STATIC_FRAMES 帧认为机器人静止，用于估计基线
        self._N_STATIC_FRAMES = 100

        # 时间戳
        self._last_imu_time = None
        self._last_publish_time = None

        # 传感器数据缓存（等待 IMU + 关节同步）
        self._imu_acc = None
        self._imu_omega = None
        self._imu_quat = None  # 缓存 IMU 四元数用于初始化 yaw
        self._imu_time = None
        self._q_all = None
        self._dq_all = None
        self._tau_all = None
        self._joint_time = None

        # 重复帧检测
        self._last_joint_time = None

        # 统计
        self._frame_count = 0
        self._update_count = 0

        # ============================================================
        # 发布器
        # ============================================================
        self._odom_pub = self.create_publisher(
            Odometry, '/odom', 10)
        self._tf_broadcaster = TransformBroadcaster(self) if self._publish_tf else None

        # ============================================================
        # 订阅器（使用传感器数据 QoS）
        # ============================================================
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        self._imu_sub = self.create_subscription(
            Imu, '/bpx/imu/data_raw', self._imu_callback, sensor_qos)
        self._joint_sub = self.create_subscription(
            JointState, '/bpx/joint_states', self._joint_callback, sensor_qos)
        self._torque_sub = self.create_subscription(
            Float64MultiArray, '/bpx/joint_torques', self._torque_callback, sensor_qos)

        # 定时发布（确保即使消息到达不规律也能以稳定频率输出）
        pub_rate = self.get_parameter('publish_rate').value
        self._pub_timer = self.create_timer(1.0 / pub_rate, self._publish_odom)

        self.get_logger().info(
            f"EKF Odom 节点已启动\n"
            f"  订阅: /bpx/imu/data_raw, /bpx/joint_states, /bpx/joint_torques\n"
            f"  发布: /bpx/odom, tf({self._odom_frame}→{self._base_frame})\n"
            f"  参数: height_lock={self._height_lock}, R_pz={self._R_pz}, "
            f"use_sqrt={self._use_sqrt}"
        )

    # ============================================================
    # 回调函数
    # ============================================================

    def _imu_callback(self, msg: Imu):
        """IMU 数据回调：缓存加速度、角速度和四元数。"""
        self._imu_acc = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ])
        self._imu_omega = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ])
        self._imu_quat = msg.orientation  # 缓存四元数用于初始化 yaw
        self._imu_time = self._to_seconds(msg.header.stamp)
        self._try_ekf_step()

    def _joint_callback(self, msg: JointState):
        """关节状态回调：提取关节角度、角速度，以及力矩（来自 effort 字段）。"""
        q = np.zeros(12)
        dq = np.zeros(12)
        tau = np.zeros(12)
        has_effort = len(msg.effort) >= len(msg.name)

        for i, name in enumerate(msg.name):
            if name in JOINT_NAME_TO_IDX:
                idx = JOINT_NAME_TO_IDX[name]
                if i < len(msg.position):
                    q[idx] = msg.position[i]
                if i < len(msg.velocity):
                    dq[idx] = msg.velocity[i]
                if has_effort:
                    tau[idx] = msg.effort[i]

        self._q_all = q
        self._dq_all = dq

        # 优先使用 effort 字段中的力矩；若驱动不发 effort，保留上一次力矩话题的值
        if has_effort and np.any(tau != 0):
            self._tau_all = tau
            if not self._tau_baseline_ready:
                self._tau_static_buf.append(tau.copy())
                if len(self._tau_static_buf) >= self._N_STATIC_FRAMES:
                    self._compute_tau_baseline()

        self._joint_time = self._to_seconds(msg.header.stamp)
        self._try_ekf_step()

    def _torque_callback(self, msg: Float64MultiArray):
        """关节力矩回调（可选）。"""
        if len(msg.data) < 12:
            return
        tau = np.array(msg.data[:12])
        self._tau_all = tau

        # 积累静态段力矩用于基线估计
        if not self._tau_baseline_ready:
            self._tau_static_buf.append(tau.copy())
            if len(self._tau_static_buf) >= self._N_STATIC_FRAMES:
                self._compute_tau_baseline()

    def _compute_tau_baseline(self):
        """从缓冲的静态帧计算力矩基线（与 compute_torque_baseline 逻辑一致）。"""
        tau_data = np.array(self._tau_static_buf)   # (N, 12)
        baseline = np.zeros((4, 3))
        for i, leg_name in enumerate(['FL', 'FR', 'HL', 'HR']):
            idx = LEG_JOINT_MAP[leg_name]
            baseline[i] = np.mean(tau_data[:, idx], axis=0)
        self._tau_baseline = baseline
        self._tau_baseline_ready = True
        self.get_logger().info(
            f"力矩基线已计算（{len(self._tau_static_buf)} 帧）: "
            + str(np.round(self._tau_baseline, 4))
        )

    # ============================================================
    # EKF 核心逻辑
    # ============================================================

    def _try_ekf_step(self):
        """
        尝试执行一步 EKF：当 IMU 和关节数据都到达时触发。

        首次触发时初始化姿态和重力估计。
        """
        if self._imu_time is None or self._joint_time is None:
            return  # 等待所有传感器数据

        if not self._initialized:
            self._initialize_ekf()
            return

        # 计算 dt
        if self._last_imu_time is None:
            self._last_imu_time = self._imu_time
            return

        dt = self._imu_time - self._last_imu_time
        self._last_imu_time = self._imu_time

        # 保护：dt 异常时跳过
        if dt <= 0 or dt > 0.5:
            return

        # 默认力矩（如果未订阅力矩话题）
        if self._tau_all is None:
            self._tau_all = np.zeros(12)

        # ============================================================
        # 1. 预测步（IMU 积分）
        # ============================================================
        omega_c = self._imu_omega - self._x[9:12]
        self._x, self._P, self._R_body = predict(
            self._x, self._P, self._imu_acc, self._imu_omega, dt,
            self._R_body, gravity=self._gravity,
        )

        # ============================================================
        # 2. 正运动学 + 触地检测 + 腿式里程计
        # ============================================================
        feet = compute_all_feet(self._q_all, self._dq_all)
        conf = detect_all_legs(
            self._tau_all, self._tau_baseline, omega_c, self._imu_acc,
            feet, self._R_body, self._x[3:6],
        )
        w_odom = support_weights_for_odometry(conf)
        v_meas, wsum = leg_odometry_velocity(
            self._q_all, self._dq_all, self._R_body, omega_c, w_odom,
        )

        # ============================================================
        # 3. 速度更新（门控）
        # ============================================================
        touched = False
        if wsum > self._wsum_min:
            R_meas = compute_velocity_meas_noise(
                wsum, R_base=R_BASE_VEL, use_sqrt=self._use_sqrt,
            )

            # 重复帧检测：若关节数据未更新，放大观测噪声
            if (self._last_joint_time is not None
                    and self._joint_time == self._last_joint_time
                    and self._repeat_R_factor > 1.0):
                R_meas = R_meas * self._repeat_R_factor

            # innovation 软门（兜极端离群）
            z_innov = v_meas - self._x[3:6]
            S = self._P[3:6, 3:6] + R_meas
            try:
                d2 = float(z_innov @ np.linalg.solve(S, z_innov))
            except np.linalg.LinAlgError:
                d2 = np.inf
            if d2 > self._chi2_soft:
                R_meas = R_meas * (d2 / self._chi2_soft)

            self._x, self._P = update_velocity(self._x, self._P, v_meas, R_meas)
            touched = True
            self._update_count += 1

        # ============================================================
        # 4. 竖直弱约束
        # ============================================================
        if self._height_lock and wsum > self._wsum_min:
            self._x, self._P = update_height(
                self._x, self._P, pz_ref=self._pz_ref, R_pz=self._R_pz,
            )
            touched = True

        # ============================================================
        # 5. 注入 + reset
        # ============================================================
        if touched:
            self._R_body = inject_attitude(self._x, self._R_body)
            self._x, self._P = reset_jacobian(self._x, self._P)

        # 记录本次关节时间戳，供下一帧重复帧检测
        self._last_joint_time = self._joint_time

        self._frame_count += 1

    def _initialize_ekf(self):
        """
        首次初始化 EKF：
          - 优先从 IMU 四元数（msg.orientation）设置初始姿态（含 yaw）
          - 回退：用 IMU 加速度估计初始 roll/pitch，yaw=0
          - 计算力矩基线
        """
        acc = self._imu_acc
        acc_norm = np.linalg.norm(acc)
        if acc_norm < 1.0:
            self.get_logger().warn("IMU 加速度模长异常，跳过初始化")
            return

        # ── 方案 A（推荐）：从 IMU 四元数直接初始化 ──────────────────
        # csv_player 已发布 imu_quat_w/x/y/z，msg.orientation 可用
        q = self._imu_quat
        if q is not None and abs(q.w) > 0.1:  # 四元数有效
            qw, qx, qy, qz = q.w, q.x, q.y, q.z
            # 四元数 → 旋转矩阵（世界系）
            self._R_body = np.array([
                [1 - 2*(qy*qy + qz*qz),   2*(qx*qy - qw*qz),     2*(qx*qz + qw*qy)],
                [2*(qx*qy + qw*qz),       1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qw*qx)],
                [2*(qx*qz - qw*qy),       2*(qy*qz + qw*qx),     1 - 2*(qx*qx + qy*qy)],
            ])
            # 提取 rpy 用于日志
            pitch = np.arcsin(-np.clip(self._R_body[2, 0], -1.0, 1.0))
            roll = np.arctan2(self._R_body[2, 1], self._R_body[2, 2])
            yaw = np.arctan2(self._R_body[1, 0], self._R_body[0, 0])
            init_method = "IMU 四元数"
        else:
            # ── 方案 B（回退）：加速度计估计 roll/pitch，yaw=0 ──────
            roll = np.arctan2(acc[1], acc[2])
            pitch = np.arctan2(-acc[0], np.sqrt(acc[1]**2 + acc[2]**2))
            yaw = 0.0

            cr, sr = np.cos(roll), np.sin(roll)
            cp, sp = np.cos(pitch), np.sin(pitch)
            cy, sy = np.cos(yaw), np.sin(yaw)
            Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
            Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
            self._R_body = Rz @ Ry @ Rx
            init_method = "加速度计（yaw=0）"

        # 估计重力大小
        g_est = acc_norm
        if 9.0 < g_est < 10.5:
            self._gravity = g_est
        else:
            self._gravity = GRAVITY

        self._initialized = True
        self._last_imu_time = self._imu_time

        self.get_logger().info(
            f"EKF 初始化完成（{init_method}）: "
            f"roll={np.degrees(roll):.1f}°, "
            f"pitch={np.degrees(pitch):.1f}°, "
            f"yaw={np.degrees(yaw):.1f}°, "
            f"g_est={self._gravity:.3f} m/s²"
        )

    # ============================================================
    # 发布
    # ============================================================

    def _publish_odom(self):
        """定时发布 Odometry 消息和 tf。"""
        if not self._initialized:
            return

        now = self.get_clock().now()

        # ============================================================
        # 构建 Odometry 消息
        # ============================================================
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = self._odom_frame
        odom_msg.child_frame_id = self._base_frame

        # --- pose ---
        qw, qx, qy, qz = R_to_quat(self._R_body)
        odom_msg.pose.pose.position.x = float(self._x[0])
        odom_msg.pose.pose.position.y = float(self._x[1])
        odom_msg.pose.pose.position.z = float(self._x[2])
        odom_msg.pose.pose.orientation = Quaternion(w=qw, x=qx, y=qy, z=qz)

        # --- pose covariance ---
        pose_cov = extract_pose_covariance(self._P)
        odom_msg.pose.covariance = tuple(pose_cov.tolist())

        # --- twist ---
        odom_msg.twist.twist.linear.x = float(self._x[3])
        odom_msg.twist.twist.linear.y = float(self._x[4])
        odom_msg.twist.twist.linear.z = float(self._x[5])
        # 角速度从 IMU 直接取（去零偏后）
        if self._imu_omega is not None:
            bg = self._x[9:12]
            odom_msg.twist.twist.angular.x = float(self._imu_omega[0] - bg[0])
            odom_msg.twist.twist.angular.y = float(self._imu_omega[1] - bg[1])
            odom_msg.twist.twist.angular.z = float(self._imu_omega[2] - bg[2])

        # --- twist covariance ---
        twist_cov = extract_twist_covariance(self._P)
        odom_msg.twist.covariance = tuple(twist_cov.tolist())

        self._odom_pub.publish(odom_msg)

        # ============================================================
        # 发布 tf: odom → base_link
        # ============================================================
        if self._tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self._odom_frame
            t.child_frame_id = self._base_frame
            t.transform.translation.x = float(self._x[0])
            t.transform.translation.y = float(self._x[1])
            t.transform.translation.z = float(self._x[2])
            t.transform.rotation = Quaternion(w=qw, x=qx, y=qy, z=qz)
            self._tf_broadcaster.sendTransform(t)

    # ============================================================
    # 工具
    # ============================================================

    @staticmethod
    def _to_seconds(stamp) -> float:
        """将 ROS2 Time 转为浮点秒数。"""
        return stamp.sec + stamp.nanosec * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = EKFOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()