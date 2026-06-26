"""
adaptive_obs.py - 观测模型与自适应观测噪声（B 路精简版）

职责边界：
  本文件只负责"观测模型"——H 矩阵、观测噪声 R、残差 z。
  EKF 的 predict / inject / reset / 主循环在 ekf_core.py。

B 路（当前使用）观测方程：
  腿式里程计 leg_odometry_velocity 先用触地腿零速约束反解出机身速度
  v_meas（世界系），作为单个 3 维速度观测喂 EKF：
      z = v_meas - v_est ,   H = [0₃ | I₃ | 0₃ | 0₃ | 0₃]
  姿态块 = 0：腿不直接修姿态（姿态靠 IMU + 速度-姿态互相关间接修）。

为什么速度块是 I（而不是 -R_body）：
  状态速度 v 是世界系，h(x)=v_foot_world 中 v_body_world 直接相加，
  ∂h/∂v = I。负号只属于残差 z = 0 - v_est，不属于雅可比。

保留的 A 路函数（build_zupt_observation_full / compute_zupt_residual）：
  B 路不调用，仅供 ekf_core 的 A 路分支与离线分析使用，已通过 FD 验证。

姿态误差约定：右乘（body 局部误差）R ← R̂·Exp([δθ]×)，与 ekf_core 一致。
"""

import numpy as np
from typing import Dict

from .config import EPSILON, R_BASE_VEL


# ------------------------------------------------------------
# 小工具
# ------------------------------------------------------------
def skew(v: np.ndarray) -> np.ndarray:
    """3 维向量的反对称矩阵 [v]×"""
    return np.array([
        [0.0, -v[2],  v[1]],
        [v[2],  0.0, -v[0]],
        [-v[1], v[0],  0.0],
    ])


# ============================================================
# 1. B 路：融合速度观测的自适应噪声
# ============================================================
def compute_velocity_meas_noise(weight_sum: float,
                                R_base: float = R_BASE_VEL,
                                weight_floor: float = 0.05,
                                use_sqrt: bool = True) -> np.ndarray:
    """
    B 路单一速度观测的自适应噪声协方差 R (3×3)。

    原理：
      leg_odometry_velocity 返回的 weight_sum 是参与融合的支撑腿权重之和。
      支撑腿越多、置信度越高 → weight_sum 越大 → 观测越可信 → R 越小。

      2026-06-24 改进：使用 √N 规律而非 N 规律。
        观测可信度提升通常服从 √N 规律（独立观测融合），
        因此 R = R_base / sqrt(weight_sum) 比 R_base / weight_sum 更稳定。
        避免 weight_sum 从 1→3 时 R 变化 3 倍过于剧烈。

        use_sqrt=True:  R = R_base / sqrt(max(w, floor))
        use_sqrt=False: R = R_base / max(w, floor)  （旧行为，保留兼容）

    weight_floor 防止 weight_sum 很小时 R 爆炸（此时本就该靠 skip 跳过更新，
    floor 只是兜底，避免数值问题）。

    参数：
        weight_sum: leg_odometry_velocity 返回的总权重
        R_base:     基础速度观测方差 (m/s)²
        weight_floor: weight_sum 下限
        use_sqrt:   是否使用 sqrt 缩放（默认 True，推荐）

    返回：
        R: (3, 3) 观测噪声协方差
    """
    w = max(float(weight_sum), weight_floor)
    if use_sqrt:
        return np.eye(3) * (R_base / np.sqrt(w))
    else:
        return np.eye(3) * (R_base / w)


# ============================================================
# 2. 逐腿自适应噪声（A 路用，保留）
# ============================================================
def compute_observation_noise(contact_conf: Dict[str, float],
                              R_base: float = R_BASE_VEL) -> Dict[str, np.ndarray]:
    """
    逐腿自适应观测噪声（A 路）：R_i = R_base / (c_i + ε) · I₃。
    置信度越高 R 越小。B 路不调用，保留供 A 路。
    """
    return {leg: np.eye(3) * (R_base / (conf + EPSILON))
            for leg, conf in contact_conf.items()}


# ============================================================
# 3. B 路观测矩阵 H = [0 | I | 0 | 0 | 0]
# ============================================================
def build_zupt_observation(leg_name: str,
                           p_foot_body: np.ndarray,
                           R_body: np.ndarray,
                           omega_body: np.ndarray) -> np.ndarray:
    """
    B 路 / 简化版 ZUPT 观测矩阵 H (3×15)，仅速度块 = I。
    （其余参数仅为保持与 A 路同签名，B 路用不到。）
    """
    H = np.zeros((3, 15))
    H[:, 3:6] = np.eye(3)        # ∂h/∂v = I（世界系速度）
    return H


# ============================================================
# 4. A 路完整版 H（含姿态耦合，保留，已 FD 验证）
# ============================================================
def build_zupt_observation_full(leg_name: str,
                                p_foot_body: np.ndarray,
                                R_body: np.ndarray,
                                omega_body: np.ndarray,
                                jacobian: np.ndarray,
                                dq_leg: np.ndarray) -> np.ndarray:
    """
    A 路完整版 ZUPT 观测矩阵 H (3×15)，含姿态误差耦合（右乘约定）。
      速度块: ∂h/∂v   = I
      姿态块: ∂h/∂δθ = -R_body · [J·dq + ω×p]×
    B 路不调用。若 EKF 改用左乘约定，姿态块需改为 -[R_body·(J·dq+ω×p)]×。
    """
    H = np.zeros((3, 15))
    H[:, 3:6] = np.eye(3)
    u = jacobian @ dq_leg + np.cross(omega_body, p_foot_body)
    H[:, 6:9] = -R_body @ skew(u)
    return H


# ============================================================
# 5. ZUPT 残差（A 路用，保留）
# ============================================================
def compute_zupt_residual(v_body_world: np.ndarray,
                          R_body: np.ndarray,
                          p_foot_body: np.ndarray,
                          v_foot_body: np.ndarray,
                          omega_body: np.ndarray) -> np.ndarray:
    """
    A 路逐腿 ZUPT 残差：z = 0 - v_foot_world_est
      = -(v_body_world + R_body·(v_foot_body + ω×p))
    负号在残差中，不在 H 中。B 路用 z = v_meas - v_est（在 ekf_core 内算）。
    """
    v_foot_world_est = v_body_world + R_body @ (
        v_foot_body + np.cross(omega_body, p_foot_body))
    return -v_foot_world_est


# ============================================================
# 自检
# ============================================================
if __name__ == "__main__":
    print("=" * 56)
    print("  adaptive_obs (B 路精简版) 自检")
    print("=" * 56)

    # [1] B 路 H：速度块 = I，姿态块 = 0
    H = build_zupt_observation('FL', np.zeros(3), np.eye(3), np.zeros(3))
    assert np.allclose(H[:, 3:6], np.eye(3)), "速度块应为 I"
    assert np.allclose(H[:, 6:9], 0), "B 路姿态块应为 0"
    print("[1] B 路 H = [0|I|0|0|0]  ✓")

    # [2] 自适应噪声（sqrt 模式）：weight_sum 越大 R 越小，且单调
    R_big = compute_velocity_meas_noise(2.0, use_sqrt=True)[0, 0]
    R_small = compute_velocity_meas_noise(0.2, use_sqrt=True)[0, 0]
    assert R_big < R_small, "weight_sum 大 → R 小"
    print(f"[2] R_sqrt(w=2.0)={R_big:.4f} < R_sqrt(w=0.2)={R_small:.4f}  ✓")

    # [3] floor 兜底：weight_sum→0 不爆炸
    R0 = compute_velocity_meas_noise(0.0, use_sqrt=True)[0, 0]
    assert np.isfinite(R0) and R0 > 0, "weight_sum=0 时 R 应有限"
    print(f"[3] R_sqrt(w=0)={R0:.4f}（floor 兜底，有限）  ✓")

    # [3b] sqrt 模式 vs 旧模式：sqrt 模式变化更平缓
    R_old_1 = compute_velocity_meas_noise(1.0, use_sqrt=False)[0, 0]
    R_old_3 = compute_velocity_meas_noise(3.0, use_sqrt=False)[0, 0]
    R_new_1 = compute_velocity_meas_noise(1.0, use_sqrt=True)[0, 0]
    R_new_3 = compute_velocity_meas_noise(3.0, use_sqrt=True)[0, 0]
    ratio_old = R_old_1 / R_old_3  # ≈3
    ratio_new = R_new_1 / R_new_3  # ≈√3≈1.73
    assert ratio_old > ratio_new, "sqrt 模式变化应更平缓"
    print(f"[3b] 旧模式 R(1)/R(3)={ratio_old:.2f}×, sqrt 模式={ratio_new:.2f}×（更稳定）  ✓")

    # [4] A 路完整版 H 速度块仍为 I（回归保护）
    Hf = build_zupt_observation_full(
        'FL', np.array([0.3, 0.1, -0.4]), np.eye(3),
        np.array([0, 0, 0.5]), np.eye(3) * 0.1, np.array([0.1, -0.5, 1.0]))
    assert np.allclose(Hf[:, 3:6], np.eye(3)), "A 路速度块应为 I"
    assert not np.allclose(Hf[:, 6:9], 0), "A 路姿态块应非零"
    print("[4] A 路完整版 H：速度块=I、姿态块≠0  ✓")

    print("\n[完成] 自检通过")