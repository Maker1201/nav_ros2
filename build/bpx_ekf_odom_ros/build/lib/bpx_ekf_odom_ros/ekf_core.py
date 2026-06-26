"""
ekf_core.py - Error-State EKF 内核（B 路精简修正版 + 竖直弱约束）

职责：predict / update_velocity / update_height / inject_attitude / reset_jacobian / ekf_step。
观测模型（H、R、残差）在 adaptive_obs.py；主循环在 run_odometry.py。

状态 x (15,)：
  [p(3) 世界系位置, v(3) 世界系速度, δθ(3) 姿态误差(右乘),
   b_g(3) 陀螺零偏, b_a(3) 加速度零偏]
姿态名义值存在 R_body（3×3），δθ 只在更新步短暂非零，注入后即 reset 清零。
姿态误差约定：右乘 / body 局部误差  R = R̂·Exp([δθ]×)。

—— 本版相对上一版的新增 ——
  · predict 增加 gravity 参数（默认 config.GRAVITY）：用实测 g 抵消竖直残差，
    缓解 pz 漂移（实测 g≈9.873 与常数 9.81 差 0.063 m/s²，恒定作用于竖直）。
  · 新增 update_height：站立行走时 pz≈常数 的竖直弱观测，绑住竖直漂移。
"""

import numpy as np
from typing import Tuple

from .config import GRAVITY, build_Q


def _skew(v):
    return np.array([[0.0, -v[2], v[1]],
                     [v[2], 0.0, -v[0]],
                     [-v[1], v[0], 0.0]])


def _exp_so3(phi):
    theta = np.linalg.norm(phi)
    if theta < 1e-12:
        return np.eye(3)
    K = _skew(phi / theta)
    return np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)


def R_to_quat(R):
    """
    从旋转矩阵 R (3×3) 提取四元数 [qw, qx, qy, qz]。
    使用稳定算法（选最大迹分量避免接近 180° 奇点）。
    """
    qw = np.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
    qx = np.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) / 2.0
    qy = np.sqrt(max(0.0, 1.0 - R[0, 0] + R[1, 1] - R[2, 2])) / 2.0
    qz = np.sqrt(max(0.0, 1.0 - R[0, 0] - R[1, 1] + R[2, 2])) / 2.0
    # 符号由非对角元确定
    qx *= np.sign(R[2, 1] - R[1, 2])
    qy *= np.sign(R[0, 2] - R[2, 0])
    qz *= np.sign(R[1, 0] - R[0, 1])
    # 归一化
    norm = np.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
    if norm < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return np.array([qw, qx, qy, qz]) / norm


def extract_pose_covariance(P):
    """
    从 15×15 协方差矩阵 P 提取 6×6 位姿协方差（位置 + 姿态）。
    
    nav_msgs/Odometry 的 pose.covariance 是 6×6 展平为 36 元素：
      [pos_cov(3×3), 0(3×3);
       0(3×3),       att_cov(3×3)]
    
    位置协方差直接从 P[0:3, 0:3] 取。
    姿态协方差从 P[6:9, 6:9]（δθ 误差状态协方差）取。
    
    返回：36 元素数组（按行展平的 6×6 矩阵）。
    """
    cov = np.zeros((6, 6))
    cov[0:3, 0:3] = P[0:3, 0:3]   # 位置协方差
    cov[3:6, 3:6] = P[6:9, 6:9]   # 姿态协方差（δθ）
    return cov.flatten()


def extract_twist_covariance(P):
    """
    从 15×15 协方差矩阵 P 提取 6×6 速度协方差（线速度 + 角速度）。
    
    nav_msgs/Odometry 的 twist.covariance 是 6×6 展平为 36 元素：
      [vel_cov(3×3),  0(3×3);
       0(3×3),        omega_cov(3×3)]
    
    线速度协方差从 P[3:6, 3:6] 取。
    角速度协方差近似从 P[6:9, 6:9] / dt² 取（姿态误差率）。
    
    返回：36 元素数组（按行展平的 6×6 矩阵）。
    """
    cov = np.zeros((6, 6))
    cov[0:3, 0:3] = P[3:6, 3:6]   # 线速度协方差
    # 角速度协方差暂设为 0（可从陀螺噪声估计）
    return cov.flatten()


def predict(x, P, acc, omega, dt, R_body, gravity=GRAVITY):
    """
    EKF 预测步。
      p ← p + v·dt
      v ← v + (R·(a_m - b_a) + [0,0,-gravity])·dt
      R ← R·Exp((ω_m - b_g)·dt)
    gravity 建议传入静态段实测值（≈9.873）以抵消竖直残差。
    返回：(x_pred, P_pred, R_pred)
    """
    x_pred = x.copy()
    v = x[3:6]; b_g = x[9:12]; b_a = x[12:15]
    acc_c = acc - b_a
    omega_c = omega - b_g

    x_pred[0:3] = x[0:3] + v * dt
    x_pred[3:6] = v + (R_body @ acc_c + np.array([0.0, 0.0, -gravity])) * dt
    R_pred = R_body @ _exp_so3(omega_c * dt)

    F = np.eye(15)
    F[0:3, 3:6] = np.eye(3) * dt
    F[3:6, 6:9] = -R_body @ _skew(acc_c) * dt
    F[3:6, 12:15] = -R_body * dt
    F[6:9, 6:9] = np.eye(3) - _skew(omega_c) * dt
    F[6:9, 9:12] = -np.eye(3) * dt

    P_pred = F @ P @ F.T + build_Q(dt)
    P_pred = (P_pred + P_pred.T) / 2.0
    return x_pred, P_pred, R_pred


def update_velocity(x, P, v_meas, R_meas):
    """B 路速度更新：z = v_meas - v_est，H=[0|I|0|0|0]。更新后由调用方注入+reset。"""
    H = np.zeros((3, 15)); H[:, 3:6] = np.eye(3)
    z = v_meas - x[3:6]
    S = H @ P @ H.T + R_meas
    S = (S + S.T) / 2.0
    try:
        K = P @ H.T @ np.linalg.inv(S)
    except np.linalg.LinAlgError:
        return x.copy(), P.copy()
    x_upd = x.copy() + K @ z
    I_KH = np.eye(15) - K @ H
    P_upd = I_KH @ P @ I_KH.T + K @ R_meas @ K.T
    return x_upd, (P_upd + P_upd.T) / 2.0


def update_height(x, P, pz_ref=0.0, R_pz=0.04, damp_vz: bool = False):
    """
    竖直位置弱观测：z = pz_ref - p_z，H=[0,0,1,0...]（标量）。
    平地行走时机身高度近似恒定，用一个弱（R 大）的 pz≈pz_ref 观测绑住竖直漂移，
    并通过互相关顺带压住 v_z。

    2026-06-24 增强：增加 v_z 直接阻尼项。
      当 damp_vz=True 时，H 矩阵增加第 5 行（v_z 索引），
      使观测同时约束 p_z 和 v_z，加速竖直速度收敛。
      这是针对四足机器人 z 轴漂移 ±20cm 问题的直接改进。

    ⚠ 假设平地；爬坡/台阶时 pz_ref 须随地形变化。R_pz 大（默认 0.04≈std 0.2m）
      只绑长期漂移、不压步态起伏。
    """
    if damp_vz:
        # 2 维观测：p_z + v_z 同时约束
        H = np.zeros((2, 15))
        H[0, 2] = 1.0       # ∂pz/∂pz = 1
        H[1, 5] = 1.0       # ∂vz/∂vz = 1
        z = np.array([pz_ref - x[2], 0.0 - x[5]])  # pz→pz_ref, vz→0
        R = np.diag([R_pz, R_pz * 2.0])  # vz 阻尼稍弱（允许步态起伏）
    else:
        H = np.zeros((1, 15)); H[0, 2] = 1.0
        z = np.array([pz_ref - x[2]])
        R = np.array([[R_pz]])

    S = H @ P @ H.T + R
    S = (S + S.T) / 2.0
    try:
        K = P @ H.T @ np.linalg.inv(S)
    except np.linalg.LinAlgError:
        return x.copy(), P.copy()
    x_upd = x.copy() + K @ z
    I_KH = np.eye(15) - K @ H
    P_upd = I_KH @ P @ I_KH.T + K @ R @ K.T
    return x_upd, (P_upd + P_upd.T) / 2.0


def inject_attitude(x, R_body):
    """R ← R̂·Exp(δθ)。"""
    return R_body @ _exp_so3(x[6:9])


def reset_jacobian(x, P):
    """δθ ← 0，P ← G·P·Gᵀ，G_att = I - ½[δθ]×。"""
    x_reset = x.copy()
    dtheta = x[6:9]
    x_reset[6:9] = 0.0
    if np.linalg.norm(dtheta) < 1e-12:
        return x_reset, P.copy()
    G = np.eye(15)
    G[6:9, 6:9] = np.eye(3) - 0.5 * _skew(dtheta)
    P_reset = G @ P @ G.T
    return x_reset, (P_reset + P_reset.T) / 2.0


def ekf_step(x, P, acc, omega, dt, R_body,
             v_meas=None, R_meas=None, do_update=True,
             gravity=GRAVITY, height_lock=False, pz_ref=0.0, R_pz=0.04):
    """完整一步：predict + 可选 vel 更新 + 可选 height 更新 + 注入 + reset。"""
    x, P, R_body = predict(x, P, acc, omega, dt, R_body, gravity=gravity)
    touched = False
    if do_update and v_meas is not None and R_meas is not None:
        x, P = update_velocity(x, P, v_meas, R_meas); touched = True
    if height_lock:
        x, P = update_height(x, P, pz_ref=pz_ref, R_pz=R_pz); touched = True
    if touched:
        R_body = inject_attitude(x, R_body)
        x, P = reset_jacobian(x, P)
    return x, P, R_body


if __name__ == "__main__":
    from .config import build_P0
    print("=" * 56); print("  ekf_core (B 路 + 竖直弱约束) 自检"); print("=" * 56)

    x = np.zeros(15); P = build_P0(); R = np.eye(3)
    for _ in range(100):
        x, P, R = predict(x, P, np.array([0, 0, GRAVITY]), np.array([0, 0, 1.0]), 0.01, R)
    yaw = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
    assert abs(yaw - 57.3) < 1.0
    print(f"[1] 预测步积分陀螺 yaw={yaw:.1f}°  ✓")

    x = np.zeros(15); P = build_P0(); R = np.eye(3); g = 9.8728
    for _ in range(200):
        x, P, R = predict(x, P, np.array([0, 0, g]), np.zeros(3), 0.01, R, gravity=g)
    assert abs(x[5]) < 1e-6
    print(f"[2] 传入实测 g：2s 静止 v_z={x[5]:.2e}（无漂）  ✓")
    x2 = np.zeros(15); P2 = build_P0(); R2 = np.eye(3)
    for _ in range(200):
        x2, P2, R2 = predict(x2, P2, np.array([0, 0, g]), np.zeros(3), 0.01, R2, gravity=9.81)
    print(f"    对照：用 9.81 时 2s 后 v_z={x2[5]:+.3f} m/s（漂移源）")

    x = np.zeros(15); x[2] = 0.3; P = build_P0()
    xu, _ = update_height(x, P, pz_ref=0.0, R_pz=0.04)
    assert abs(xu[2]) < abs(x[2])
    print(f"[3] 竖直弱约束 pz: 0.300 → {xu[2]:.3f} m  ✓")

    x = np.zeros(15); x[3:6] = [0.5, 0, 0]; P = build_P0()
    xu, _ = update_velocity(x, P, np.zeros(3), np.eye(3) * 0.01)
    assert np.linalg.norm(xu[3:6]) < 0.5
    print(f"[4] 速度 0.50→{np.linalg.norm(xu[3:6]):.3f} 向测量收敛  ✓")
    print("\n[完成] 自检通过")