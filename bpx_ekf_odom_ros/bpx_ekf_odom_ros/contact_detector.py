"""
contact_detector.py - 触地检测模块

两种触地置信度方法 + 加权融合：
  1. 足端速度法（主判据）：||v_foot_world|| 越小 → 触地概率越高
     - 实测区分度：支撑腿 ~0.22 m/s，摆动腿 ~0.71 m/s
     - 阈值设在 0.4 m/s，sigmoid 斜率加大
  2. 力矩基线法（辅助）：|τ - τ_baseline| 越大 → 触地概率越高

IMU 方差法仅作全局门控（不参与逐腿权重），当机身加速度方差过大时
降低所有腿的置信度（提示可能处于跳跃/颠簸等非稳态）。

最终置信度 = 加权融合（无二次 sigmoid 压缩）
"""

import numpy as np
import pandas as pd
from typing import Dict, Optional

from .config import (
    LEG_JOINT_MAP, JOINT_TAU_COLS,
    TAU_BASELINE_WINDOW, TAU_THRESHOLD_FACTOR,
    ALPHA_TORQUE, ALPHA_CONSISTENCY,
    CONTACT_THRESHOLD, W_TORQUE, W_CONSISTENCY,
    EPSILON, GRAVITY,
    ODOM_SUPPORT_THRESHOLD, ODOM_WEIGHT_POWER,
)
from .kinematics import foot_velocity_in_world


def compute_torque_baseline(static_df: pd.DataFrame) -> np.ndarray:
    """
    从静态段计算力矩基线（每条腿3个关节的力矩均值）。

    参数：
        static_df: 静态段 DataFrame（cmd_phase==2）

    返回：
        tau_baseline: (4, 3) 每条腿3个关节的力矩均值
                      顺序：[FL, FR, HL, HR]
    """
    tau_cols = JOINT_TAU_COLS
    if not all(c in static_df.columns for c in tau_cols):
        raise ValueError("静态段缺少力矩列")

    tau_data = static_df[tau_cols].values  # (N, 12)
    baseline = np.zeros((4, 3))

    for i, leg_name in enumerate(['FL', 'FR', 'HL', 'HR']):
        idx = LEG_JOINT_MAP[leg_name]
        baseline[i] = np.mean(tau_data[:, idx], axis=0)

    return baseline


def contact_from_torque(tau_all: np.ndarray,
                        tau_baseline: np.ndarray,
                        threshold: float = 1.0) -> np.ndarray:
    """
    基于力矩的触地置信度。

    原理：支撑腿承受机身重量，力矩绝对值大；摆动腿仅克服自重，力矩小。
    使用 |τ - τ_baseline| 相对变化量，消除静态段基线。
    取该腿3个关节的力矩绝对值之和（比均值更灵敏）。

    参数：
        tau_all: (12,) 所有关节力矩
        tau_baseline: (4, 3) 力矩基线
        threshold: 力矩阈值

    返回：
        conf: (4,) 每条腿的触地置信度 [0, 1]
    """
    conf = np.zeros(4)
    for i, leg_name in enumerate(['FL', 'FR', 'HL', 'HR']):
        idx = LEG_JOINT_MAP[leg_name]
        tau_leg = tau_all[idx]
        tau_base = tau_baseline[i]

        # 相对力矩变化量（取3个关节的绝对值之和，比均值更灵敏）
        delta = np.sum(np.abs(tau_leg - tau_base))
        # sigmoid 映射到 [0, 1]
        conf[i] = _sigmoid(delta - threshold, alpha=ALPHA_TORQUE)

    return conf


def contact_from_consistency(v_foot_world: np.ndarray) -> float:
    """
    基于足端速度一致性的触地置信度（主判据）。

    原理：支撑腿足端相对地面静止（速度 ≈ 0），摆动腿足端有运动速度。
    实测区分度：支撑腿 ~0.22 m/s，摆动腿 ~0.71 m/s。
    用 ||v_foot_world|| 衡量：速度越小 → 触地概率越高。

    参数：
        v_foot_world: (3,) 足端在世界系下的速度

    返回：
        conf: 该腿的触地置信度 [0, 1]
    """
    foot_speed = np.linalg.norm(v_foot_world)
    # 阈值 0.4 m/s（介于支撑/摆动实测均值之间），斜率加大
    conf = _sigmoid(-foot_speed + 0.4, alpha=ALPHA_CONSISTENCY * 2.0)
    return conf


def _sigmoid(x: float, alpha: float = 1.0) -> float:
    """sigmoid 函数（对输入 clip 防止 np.exp 溢出告警）"""
    z = np.clip(alpha * x, -60.0, 60.0)
    return 1.0 / (1.0 + np.exp(-z))


def detect_all_legs(tau_all: np.ndarray,
                    tau_baseline: np.ndarray,
                    omega: np.ndarray,
                    acc: np.ndarray,
                    feet_data: Dict[str, Dict[str, np.ndarray]],
                    R_body: np.ndarray,
                    v_body_world: np.ndarray,
                    return_raw: bool = False) -> Dict[str, float]:
    """
    对一帧数据检测4条腿的触地置信度（两种方法融合）。

    融合策略：
      - 足端速度法（主判据，权重 0.7）：每条腿独立，区分度最高
      - 力矩基线法（辅助，权重 0.3）：每条腿独立
      - IMU 方差法仅作全局门控：当机身加速度方差过大时，整体降低置信度
        （提示可能处于跳跃/颠簸等非稳态）

    参数：
        tau_all: (12,) 所有关节力矩
        tau_baseline: (4, 3) 力矩基线
        omega: (3,) 机身角速度（机身坐标系）
        acc: (3,) 机身加速度（机身坐标系）
        feet_data: compute_all_feet() 返回的足端数据
        R_body: (3, 3) 机身在世界系下的旋转矩阵
        v_body_world: (3,) 机身在世界系下的速度
        return_raw: 如果为 True，返回包含逐腿置信度、足端速度、IMU 门控等
                    详细信息的字典，用于离线分析和调参。

    返回：
        如果 return_raw=False（默认）：
            {leg_name: contact_confidence} 每条腿的触地置信度
        如果 return_raw=True：
            {
                'conf': {leg_name: contact_confidence},  # 最终融合置信度
                'conf_speed': {leg_name: speed_conf},     # 足端速度法原始置信度
                'foot_speed': {leg_name: speed_norm},     # 足端速度模长 (m/s)
                'imu_gate': gate,                          # IMU 全局门控值
                'conf_torque': {leg_name: torque_conf},    # 力矩法原始置信度（如启用）
            }
    """
    # 1. 足端速度法（每条腿独立，唯一主判据）
    conf_speed = np.zeros(4)
    foot_speeds = np.zeros(4)
    for i, leg_name in enumerate(['FL', 'FR', 'HL', 'HR']):
        p_foot_body = feet_data[leg_name]['pos']
        v_foot_body = feet_data[leg_name]['vel']
        v_foot_world = foot_velocity_in_world(R_body, p_foot_body,
                                              v_foot_body, omega)
        foot_speeds[i] = np.linalg.norm(v_foot_world)
        conf_speed[i] = contact_from_consistency(v_foot_world)

    # 2. 逐腿融合：仅足端速度法（W_TORQUE=0，力矩法在动态行走被惯性力污染）
    #    保留 W_TORQUE 加权形式，便于将来标定好力矩法后重新启用。
    conf_fused = W_CONSISTENCY * conf_speed
    conf_torque = np.zeros(4)
    if W_TORQUE > 0.0:
        conf_torque = contact_from_torque(tau_all, tau_baseline)
        conf_fused = conf_fused + W_TORQUE * conf_torque

    # 3. IMU 全局门控（不参与逐腿区分，只在机身剧烈运动时整体降信）
    #    用 ||acc|| 相对重力的偏差 + 角速度 衡量动态程度（去掉重力常量干扰）
    imu_activity = np.mean(np.abs(omega)) + abs(np.linalg.norm(acc) - GRAVITY) * 0.1
    gate = _sigmoid(-imu_activity + 2.0, alpha=2.0)
    conf_fused = conf_fused * gate

    # 5. 裁剪到 [0, 1]（不再用二次 sigmoid 压缩）
    conf_fused = np.clip(conf_fused, 0.0, 1.0)

    result = {}
    for i, leg_name in enumerate(['FL', 'FR', 'HL', 'HR']):
        result[leg_name] = float(conf_fused[i])

    if return_raw:
        raw = {
            'conf': result,
            'conf_speed': {leg: float(conf_speed[i])
                           for i, leg in enumerate(['FL', 'FR', 'HL', 'HR'])},
            'foot_speed': {leg: float(foot_speeds[i])
                           for i, leg in enumerate(['FL', 'FR', 'HL', 'HR'])},
            'imu_gate': float(gate),
        }
        if W_TORQUE > 0.0:
            raw['conf_torque'] = {leg: float(conf_torque[i])
                                  for i, leg in enumerate(['FL', 'FR', 'HL', 'HR'])}
        return raw

    return result


def get_contact_binary(confidences: Dict[str, float],
                       threshold: float = None) -> Dict[str, bool]:
    """
    将触地置信度转为二值触地状态。

    参数：
        confidences: detect_all_legs() 返回的置信度字典
        threshold: 阈值，默认 CONTACT_THRESHOLD

    返回：
        {leg_name: is_contact}
    """
    if threshold is None:
        threshold = CONTACT_THRESHOLD
    return {leg: conf >= threshold for leg, conf in confidences.items()}


def support_weights_for_odometry(
    confidences: Dict[str, float],
    threshold: float = None,
    power: float = None,
) -> Dict[str, float]:
    """
    把连续触地置信度转成"喂给腿式里程计的支撑腿权重"。

    为什么需要单独转换：
        detect_all_legs 返回的是连续置信度（供 EKF 自适应观测噪声用），
        但摆动腿速度高达 ~1.5 m/s，哪怕带 0.15 的小权重，乘进里程计加权
        平均也会把机身速度估计往零拖。实测纯连续权重相关 0.78，硬门+锐化
        后达 0.86~0.88。

    处理：
        1. 低于 threshold 的腿权重清零（硬门，排除摆动腿）；
        2. 保留腿的权重做 power 次幂锐化（拉开支撑腿之间的差距）。

    参数：
        confidences: detect_all_legs() 返回的置信度字典
        threshold: 硬门阈值，默认 ODOM_SUPPORT_THRESHOLD
        power:     锐化幂次，默认 ODOM_WEIGHT_POWER

    返回：
        {leg_name: odom_weight}  直接传给 leg_odometry_velocity 的 contact_confidence
    """
    if threshold is None:
        threshold = ODOM_SUPPORT_THRESHOLD
    if power is None:
        power = ODOM_WEIGHT_POWER
    return {leg: (conf ** power if conf >= threshold else 0.0)
            for leg, conf in confidences.items()}


def select_support_legs_by_speed(
    feet_data: Dict[str, Dict[str, np.ndarray]],
    R_body: np.ndarray,
    omega: np.ndarray,
) -> Dict[str, float]:
    """
    兜底方案：直接按足端速度选择支撑腿。

    取足端速度最小的两条对角腿作为支撑腿（四足机器人典型 trot 步态），
    置信度设为 0.9；其余腿设为 0.1。

    参数：
        feet_data: compute_all_feet() 返回的足端数据
        R_body: (3, 3) 机身在世界系下的旋转矩阵
        omega: (3,) 机身角速度（机身坐标系）

    返回：
        {leg_name: contact_confidence}
    """
    # 计算每条腿的足端速度
    leg_speeds = {}
    for leg_name in ['FL', 'FR', 'HL', 'HR']:
        p_foot_body = feet_data[leg_name]['pos']
        v_foot_body = feet_data[leg_name]['vel']
        v_foot_world = foot_velocity_in_world(R_body, p_foot_body,
                                              v_foot_body, omega)
        leg_speeds[leg_name] = np.linalg.norm(v_foot_world)

    # trot 步态对角腿同时触地：比较两组对角对(FL+HR vs FR+HL)的足端速度和，
    # 选速度和更小的那组为支撑腿（比单纯取最慢两条更鲁棒，且强制对角约束）
    s_diag1 = leg_speeds['FL'] + leg_speeds['HR']   # 对角对 1
    s_diag2 = leg_speeds['FR'] + leg_speeds['HL']   # 对角对 2
    support = {'FL', 'HR'} if s_diag1 <= s_diag2 else {'FR', 'HL'}

    return {leg: (0.9 if leg in support else 0.1)
            for leg in ['FL', 'FR', 'HL', 'HR']}


# ============================================================
# 测试入口
# ============================================================
if __name__ == "__main__":
    print("=" * 60)
    print("  触地检测模块测试")
    print("=" * 60)

    # 模拟数据：模拟 trot 步态（FL+HR 支撑，FR+HL 摆动）
    np.random.seed(42)

    # 支撑腿（FL, HR）：足端速度小，力矩大
    # 摆动腿（FR, HL）：足端速度大，力矩小
    tau_all = np.zeros(12)
    # FL (idx 0-2): 支撑 → 大力矩
    tau_all[0:3] = [2.5, 3.0, 1.8]
    # FR (idx 3-5): 摆动 → 小力矩
    tau_all[3:6] = [0.3, 0.5, 0.2]
    # HL (idx 6-8): 摆动 → 小力矩
    tau_all[6:9] = [0.4, 0.6, 0.3]
    # HR (idx 9-11): 支撑 → 大力矩
    tau_all[9:12] = [2.8, 3.2, 2.0]

    tau_baseline = np.zeros((4, 3))  # 零基线（简化测试）
    omega = np.array([0.1, 0.05, 0.02])
    acc = np.array([0.5, 0.3, 9.8])
    R_body = np.eye(3)
    v_body_world = np.zeros(3)

    # 模拟足端数据
    from .kinematics import compute_all_feet
    q_all = np.zeros(12)
    dq_all = np.zeros(12)
    # 支撑腿（FL, HR）：小关节速度 → 足端速度小
    # 摆动腿（FR, HL）：大关节速度 → 足端速度大
    q_all[0:3] = [0.05, -0.4, 0.8]
    q_all[3:6] = [-0.05, -0.4, 0.8]
    q_all[6:9] = [0.05, -0.3, 0.7]
    q_all[9:12] = [-0.05, -0.3, 0.7]
    dq_all[0:3] = [0.1, 0.2, 0.3]     # FL: 慢速
    dq_all[3:6] = [1.5, 2.0, 2.5]     # FR: 快速（摆动）
    dq_all[6:9] = [1.2, 1.8, 2.2]     # HL: 快速（摆动）
    dq_all[9:12] = [0.1, 0.2, 0.3]    # HR: 慢速
    feet_data = compute_all_feet(q_all, dq_all)

    # 打印各腿足端速度
    print("\n[足端速度]")
    for leg_name in ['FL', 'FR', 'HL', 'HR']:
        p = feet_data[leg_name]['pos']
        v = feet_data[leg_name]['vel']
        v_world = foot_velocity_in_world(R_body, p, v, omega)
        print(f"  {leg_name}: pos={np.round(p, 3)}, "
              f"vel_body={np.round(v, 3)}, "
              f"vel_world_norm={np.linalg.norm(v_world):.3f}")

    # 测试触地检测
    conf = detect_all_legs(tau_all, tau_baseline, omega, acc,
                           feet_data, R_body, v_body_world)
    binary = get_contact_binary(conf)

    print("\n[detect_all_legs 融合结果]:")
    for leg_name in ['FL', 'FR', 'HL', 'HR']:
        print(f"  {leg_name}: 置信度={conf[leg_name]:.3f}, "
              f"触地={binary[leg_name]}")

    # 测试兜底方案
    conf2 = select_support_legs_by_speed(feet_data, R_body, omega)
    binary2 = get_contact_binary(conf2)

    print("\n[select_support_legs_by_speed 兜底结果]:")
    for leg_name in ['FL', 'FR', 'HL', 'HR']:
        print(f"  {leg_name}: 置信度={conf2[leg_name]:.3f}, "
              f"触地={binary2[leg_name]}")

    # 验证：支撑腿置信度应显著高于摆动腿
    support_legs = ['FL', 'HR']
    swing_legs = ['FR', 'HL']
    support_mean = np.mean([conf[leg] for leg in support_legs])
    swing_mean = np.mean([conf[leg] for leg in swing_legs])
    print(f"\n[区分度验证]")
    print(f"  支撑腿均值: {support_mean:.3f}")
    print(f"  摆动腿均值: {swing_mean:.3f}")
    print(f"  区分度: {support_mean - swing_mean:.3f}")
    assert support_mean > swing_mean + 0.2, \
        f"支撑腿({support_mean:.3f})应显著高于摆动腿({swing_mean:.3f})"
    print("  ✓ 区分度合格")

    print("\n[完成] 触地检测测试通过！")