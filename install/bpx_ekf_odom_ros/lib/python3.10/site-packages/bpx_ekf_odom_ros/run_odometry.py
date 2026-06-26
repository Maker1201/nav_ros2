"""
run_odometry.py - B 路里程计正循环驱动
  （每帧更新+重复帧降权 + 软门 + 竖直弱约束 + 实测重力）

数据流（每帧）：
  1. predict：IMU 积分 + 推进 R_body（用静态段实测重力 g_est）
  2. compute_all_feet → detect_all_legs → support_weights_for_odometry
  3. leg_odometry_velocity → (v_meas, weight_sum)
  4. 速度更新门控：
       · wsum > wsum_min 才更新（腾空/无支撑腿则只预测）
       · 每帧都更新（不再因 is_new_sample=False 跳过）——避免重复帧间预测漂移
         （实测：跳过重复帧会让坐下相位的速度纯预测漂移成 -2.5 尖刺）
       · 对重复帧（is_new_sample=False）放大 R（repeat_R_factor），避免重复
         观测虚增信息量导致协方差过度收缩
       · innovation 软门：d² 超阈值则再放大 R 降权（兜极端离群，不硬拒绝）
  5. 竖直弱约束（站立时 pz≈常数）→ 注入 + reset（每帧一次）

2026-06-24 新增：
  · NIS 统计：记录每帧 innovation (z)、协方差 S、NIS (d²)，输出 NIS 均值与
    卡方检验通过率（95% 分位 7.815），用于 EKF 调参。
  · 逐腿置信度记录：FL_conf, FR_conf, HL_conf, HR_conf 写入轨迹 CSV，
    便于离线分析触地检测质量。
  · use_sqrt 参数：控制 R = R_base / sqrt(weight_sum) 还是 R_base / weight_sum。

姿态约定：右乘（与 ekf_core 一致）。
用法：python3 -m bpx_ekf_odom.run_odometry [csv路径]
"""

import sys
import numpy as np
import pandas as pd
from pathlib import Path

from .config import (
    build_P0, GRAVITY, R_BASE_VEL,
    JOINT_Q_COLS, JOINT_DQ_COLS, JOINT_TAU_COLS,
    IMU_ACC_COLS, IMU_OMEGA_COLS, IMU_RPY_COLS,
    DEFAULT_CSV_PATH,
)
from .preprocessor import preprocess_pipeline
from .data_loader import get_static_segment
from .kinematics import compute_all_feet, leg_odometry_velocity
from .contact_detector import (
    detect_all_legs, support_weights_for_odometry, compute_torque_baseline,
)
from .adaptive_obs import compute_velocity_meas_noise
from .ekf_core import (
    predict, update_velocity, update_height, inject_attitude, reset_jacobian,
    R_to_quat,
)

# 自由度：NIS 卡方检验 95% 分位（3 维观测）
CHI2_95_3DOF = 7.815


def rpy_to_R(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def R_to_rpy(R):
    pitch = np.arcsin(-np.clip(R[2, 0], -1.0, 1.0))
    roll = np.arctan2(R[2, 1], R[2, 2])
    yaw = np.arctan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


def run(csv_path,
        wsum_min=0.05,            # 触发速度更新的 weight_sum 下限
        repeat_R_factor=4.0,      # 重复帧(is_new_sample=False)的 R 放大倍数
                                  #   ≈ 上采样比(96/25≈4)，抵消重复观测的虚增信息
        only_new_sample=False,    # True=老行为(跳过重复帧, 会有坐下尖刺); 默认 False
        chi2_soft=50.0,           # innovation 软门：d²超此值再放大R降权（兜极端离群）
        height_lock=True,         # 竖直弱约束开关
        R_pz=0.04,                # 竖直观测方差 m²（越大越弱）
        pz_ref=0.0,               # 参考高度（平地=初始站立高度）
        use_sqrt=True,            # R = R_base / sqrt(weight_sum)（推荐）
        record_nis=True,          # 记录 innovation/NIS 统计
        record_leg_conf=True,     # 记录逐腿置信度
        verbose=True):
    """
    跑完整条 CSV，返回轨迹 DataFrame：
      t, px,py,pz, vx,vy,vz, roll,pitch,yaw, wsum, updated
      + 可选：nis, nis_x, nis_y, nis_z（innovation 各分量）
      + 可选：FL_conf, FR_conf, HL_conf, HR_conf（逐腿置信度）
    """
    df = preprocess_pipeline(csv_path, interpolate=False, verbose=verbose)
    if len(df) == 0:
        raise ValueError(f"预处理返回空数据：{csv_path}")

    # 静态段：初始姿态 + 实测重力 + 力矩基线
    try:
        static_df = get_static_segment(df)
    except Exception:
        static_df = df.iloc[:0]

    if len(static_df) >= 10 and all(c in df.columns for c in IMU_RPY_COLS):
        rpy0 = static_df[IMU_RPY_COLS].values.mean(axis=0)
    elif all(c in df.columns for c in IMU_RPY_COLS):
        rpy0 = df[IMU_RPY_COLS].iloc[0].values
    else:
        rpy0 = np.zeros(3)
    R_body = rpy_to_R(*rpy0)

    if len(static_df) >= 10 and all(c in static_df.columns for c in IMU_ACC_COLS):
        g_est = float(np.linalg.norm(static_df[IMU_ACC_COLS].values, axis=1).mean())
        if not (9.0 < g_est < 10.5):
            g_est = GRAVITY
    else:
        g_est = GRAVITY

    try:
        tau_baseline = compute_torque_baseline(static_df)
    except Exception:
        tau_baseline = np.zeros((4, 3))

    if verbose:
        print(f"[run] 初始 rpy(静态段) = {np.round(np.degrees(rpy0), 2)} 度, "
              f"实测重力 g_est = {g_est:.4f} m/s²")
        print(f"[run] 每帧更新(repeat_R×{repeat_R_factor}), 软门={chi2_soft}, "
              f"竖直弱约束={'开' if height_lock else '关'}(R_pz={R_pz}), "
              f"use_sqrt={'开' if use_sqrt else '关'}")

    x = np.zeros(15)
    P = build_P0()

    has_tau = all(c in df.columns for c in JOINT_TAU_COLS)
    has_newflag = 'is_new_sample' in df.columns
    acc_arr = df[IMU_ACC_COLS].values
    omega_arr = df[IMU_OMEGA_COLS].values
    q_arr = df[JOINT_Q_COLS].values
    dq_arr = df[JOINT_DQ_COLS].values
    tau_arr = df[JOINT_TAU_COLS].values if has_tau else np.zeros((len(df), 12))
    dt_arr = df['dt'].values if 'dt' in df.columns else \
        np.r_[0.0, np.diff(df['timestamp_s'].values)]
    t_arr = df['timestamp_s'].values
    newflag = df['is_new_sample'].values if has_newflag else np.ones(len(df), bool)

    rows = []
    n_update = n_soft = 0
    nis_all = []  # 收集所有 NIS 值用于统计

    for k in range(len(df)):
        dt = float(dt_arr[k])
        acc = acc_arr[k]
        omega = omega_arr[k]

        if dt <= 0 or dt > 0.5:
            rows.append(_record(t_arr[k], x, P, R_body, 0.0, False,
                                nis=np.nan, leg_conf=None))
            continue

        # --- 预测 ---
        omega_c = omega - x[9:12]
        x, P, R_body = predict(x, P, acc, omega, dt, R_body, gravity=g_est)

        # --- 观测：腿式里程计速度 ---
        q_all, dq_all, tau_all = q_arr[k], dq_arr[k], tau_arr[k]
        feet = compute_all_feet(q_all, dq_all)
        conf = detect_all_legs(tau_all, tau_baseline, omega_c, acc,
                               feet, R_body, x[3:6])
        w_odom = support_weights_for_odometry(conf)
        v_meas, wsum = leg_odometry_velocity(q_all, dq_all, R_body, omega_c, w_odom)

        # --- 速度更新门控 ---
        do_vel = (wsum > wsum_min) and (newflag[k] or not only_new_sample)
        touched = False
        nis = np.nan
        if do_vel:
            R_meas = compute_velocity_meas_noise(wsum, R_base=R_BASE_VEL,
                                                 use_sqrt=use_sqrt)
            # 重复帧降权（避免虚增信息量）
            if not newflag[k]:
                R_meas = R_meas * repeat_R_factor
            # innovation 软门（兜极端离群）
            z_innov = v_meas - x[3:6]
            S = P[3:6, 3:6] + R_meas
            try:
                d2 = float(z_innov @ np.linalg.solve(S, z_innov))
            except np.linalg.LinAlgError:
                d2 = np.inf
            if d2 > chi2_soft:
                R_meas = R_meas * (d2 / chi2_soft)
                n_soft += 1
            x, P = update_velocity(x, P, v_meas, R_meas)
            touched = True
            n_update += 1
            nis = d2
            nis_all.append(d2)

        # --- 竖直弱约束（有支撑腿才用）---
        if height_lock and wsum > wsum_min:
            x, P = update_height(x, P, pz_ref=pz_ref, R_pz=R_pz)
            touched = True

        # --- 注入 + reset（每帧一次）---
        if touched:
            R_body = inject_attitude(x, R_body)
            x, P = reset_jacobian(x, P)

        rows.append(_record(t_arr[k], x, P, R_body, wsum, touched,
                            nis=nis, leg_conf=conf if record_leg_conf else None))

    traj = pd.DataFrame(rows)
    if verbose:
        nis_arr = np.array(nis_all)
        nis_mean = float(np.nanmean(nis_arr)) if len(nis_arr) > 0 else np.nan
        nis_pass = float(np.mean(nis_arr < CHI2_95_3DOF)) * 100 if len(nis_arr) > 0 else np.nan
        print(f"[run] 完成：{len(df)} 帧，速度更新 {n_update}（软门降权 {n_soft}），"
              f"时长 {t_arr[-1]-t_arr[0]:.1f}s")
        print(f"[run] 末端位置 = {np.round(traj[['px','py','pz']].iloc[-1].values, 3)} m")
        print(f"[run] vx: P5={np.percentile(traj.vx,5):.2f}, 中位={np.median(traj.vx):.3f}, "
              f"P95={np.percentile(traj.vx,95):.2f}, 极值=[{traj.vx.min():.2f},{traj.vx.max():.2f}]")
        print(f"[run] pz 范围 [{traj.pz.min():.3f}, {traj.pz.max():.3f}] m")
        if len(nis_arr) > 0:
            print(f"[run] NIS: 均值={nis_mean:.2f}, "
                  f"<χ²₉₅%(7.815)={nis_pass:.1f}%")
        if record_leg_conf:
            for leg in ['FL', 'FR', 'HL', 'HR']:
                col = f'{leg}_conf'
                if col in traj.columns:
                    print(f"[run] {leg}_conf: 均值={traj[col].mean():.3f}, "
                          f"中位={traj[col].median():.3f}")
    return traj


def _record(t, x, P, R_body, wsum, updated, nis=np.nan, leg_conf=None):
    roll, pitch, yaw = R_to_rpy(R_body)
    qw, qx, qy, qz = R_to_quat(R_body)

    rec = {
        't': t,

        # pose
        'px': x[0], 'py': x[1], 'pz': x[2],
        'vx': x[3], 'vy': x[4], 'vz': x[5],

        # attitude (RPY + quat)
        'roll': roll, 'pitch': pitch, 'yaw': yaw,
        'qw': qw, 'qx': qx, 'qy': qy, 'qz': qz,

        'wsum': wsum,
        'updated': updated,
        'nis': nis,
    }

    # =========================
    # 1) 位置协方差 3×3
    # =========================
    rec.update({
        'pos_cov_xx': P[0, 0],
        'pos_cov_xy': P[0, 1],
        'pos_cov_xz': P[0, 2],
        'pos_cov_yy': P[1, 1],
        'pos_cov_yz': P[1, 2],
        'pos_cov_zz': P[2, 2],
    })

    # =========================
    # 2) 速度协方差 3×3
    # =========================
    rec.update({
        'vel_cov_xx': P[3, 3],
        'vel_cov_xy': P[3, 4],
        'vel_cov_xz': P[3, 5],
        'vel_cov_yy': P[4, 4],
        'vel_cov_yz': P[4, 5],
        'vel_cov_zz': P[5, 5],
    })

    # =========================
    # 3) 姿态误差协方差（建议补上）
    # =========================
    rec.update({
        'att_cov_xx': P[6, 6],
        'att_cov_yy': P[7, 7],
        'att_cov_zz': P[8, 8],
    })

    # 腿置信度
    if leg_conf is not None:
        for leg in ['FL', 'FR', 'HL', 'HR']:
            rec[f'{leg}_conf'] = leg_conf.get(leg, 0.0)

    return rec


def main():
    csv_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_CSV_PATH
    traj = run(csv_path, verbose=True)
    out_csv = Path(csv_path).with_name("odom_" + Path(csv_path).stem + ".csv")
    traj.to_csv(out_csv, index=False)
    print(f"[run] 轨迹已保存：{out_csv}")


if __name__ == "__main__":
    main()
