"""
kinematics.py - 正运动学计算

功能：
  1. forward_kinematics(q_leg, leg_name) → 足端位置（机身坐标系）
  2. compute_jacobian(q_leg, leg_name) → 3×3 几何雅可比矩阵
  3. foot_velocity(jacobian, dq_leg) → 足端速度（机身坐标系）
  4. compute_all_feet(q_all, dq_all) → 4条腿的足端位置和速度
  5. foot_velocity_in_world(R_body, p_foot_body, v_foot_body, omega_body) → 世界系足端速度

BPX 每条腿 3-DOF 结构（从机身到足端）：
  HipRoll (绕X) → HipPitch (绕Y) → Knee (绕Y)

坐标系约定（机身系）：X 向前, Y 向左, Z 向上。
  - HipPitch/Knee 绕 Y 轴 → 在 X-Z 矢状面摆腿（产生前后 X 与上下 Z）
  - HipRoll 绕 X 轴 → 把竖直分量旋入 Y-Z（产生左右 Y 与上下 Z）
  正确顺序：先在 X-Z 面算 pitch/knee 足端，再用 R_x(q0) 旋转其 (Y,Z)。
"""

import numpy as np
from typing import Dict, Tuple

from .config import (
    L_THIGH, L_SHANK, HIP_OFFSET, LEG_JOINT_MAP,
    JOINT_Q_COLS, JOINT_DQ_COLS,
)


def forward_kinematics(q_leg: np.ndarray,
                       leg_name: str) -> np.ndarray:
    """
    正运动学：给定腿的3个关节角，计算足端在机身坐标系下的位置。

    参数：
        q_leg: (3,) 关节角度 [hipRoll, hipPitch, knee] (rad)
        leg_name: 腿名 'FL'/'FR'/'HL'/'HR'

    返回：
        p_foot: (3,) 足端位置（机身坐标系，单位：米）
    """
    hip_offset = HIP_OFFSET[leg_name]
    q0, q1, q2 = q_leg  # hipRoll, hipPitch, knee
    s0, c0 = np.sin(q0), np.cos(q0)

    # --- pitch + knee 在 X-Z 矢状面（HipRoll 旋转前）---
    #   x_local: 前后投影（由俯仰/膝盖摆动产生，迈步前进的来源）
    #   z_local: 竖直投影（向下为负）
    x_local = -(L_THIGH * np.sin(q1) + L_SHANK * np.sin(q1 + q2))  # 符号校准: 前进=+X
    z_local = -(L_THIGH * np.cos(q1) + L_SHANK * np.cos(q1 + q2))

    # --- HipRoll(q0) 绕 X 轴旋转 (y,z)，X 不变 ---
    # R_x(q0)·[x_local, 0, z_local]^T = [x_local, -z_local·s0, z_local·c0]
    p_foot = np.array([
        hip_offset[0] + x_local,        # X: 前后（仅来自 pitch/knee）
        hip_offset[1] - z_local * s0,   # Y: 侧摆（roll 把竖直分量旋入）
        hip_offset[2] + z_local * c0,   # Z: 上下
    ])

    return p_foot


def compute_jacobian(q_leg: np.ndarray,
                     leg_name: str) -> np.ndarray:
    """
    计算 3×3 几何雅可比矩阵 J = ∂p/∂q

    参数：
        q_leg: (3,) 关节角度 [hipRoll, hipPitch, knee] (rad)
        leg_name: 腿名（雅可比与髋偏移无关）

    返回：
        J: (3, 3) 雅可比矩阵
    """
    q0, q1, q2 = q_leg
    s0, c0 = np.sin(q0), np.cos(q0)

    # x_local 及其对 q1,q2 的偏导
    L   = L_THIGH * np.sin(q1) + L_SHANK * np.sin(q1 + q2)   # = x_local
    dL1 = L_THIGH * np.cos(q1) + L_SHANK * np.cos(q1 + q2)
    dL2 = L_SHANK * np.cos(q1 + q2)

    # z_local 及其对 q1,q2 的偏导
    Z   = -(L_THIGH * np.cos(q1) + L_SHANK * np.cos(q1 + q2))  # = z_local
    dZ1 = L_THIGH * np.sin(q1) + L_SHANK * np.sin(q1 + q2)     # = L
    dZ2 = L_SHANK * np.sin(q1 + q2)

    J = np.zeros((3, 3))

    # p_x = ox + x_local          （与 hipRoll 无关）
    J[0, 0] = 0.0
    J[0, 1] = -dL1   # 符号校准: 前进=+X
    J[0, 2] = -dL2

    # p_y = oy - z_local·sin(q0)
    J[1, 0] = -Z * c0
    J[1, 1] = -dZ1 * s0
    J[1, 2] = -dZ2 * s0

    # p_z = oz + z_local·cos(q0)
    J[2, 0] = -Z * s0
    J[2, 1] =  dZ1 * c0
    J[2, 2] =  dZ2 * c0

    return J


def foot_velocity(jacobian: np.ndarray,
                  dq_leg: np.ndarray) -> np.ndarray:
    """
    计算足端速度（机身坐标系）

    v_foot_body = J · dq_leg

    参数：
        jacobian: (3, 3) 雅可比矩阵
        dq_leg: (3,) 关节角速度 [d_hipRoll, d_hipPitch, d_knee] (rad/s)

    返回：
        v_foot: (3,) 足端速度（机身坐标系，m/s）
    """
    return jacobian @ dq_leg


def compute_all_feet(q_all: np.ndarray,
                     dq_all: np.ndarray) -> Dict[str, Dict[str, np.ndarray]]:
    """
    对一帧数据计算4条腿的足端位置和速度。

    参数：
        q_all: (12,) 所有关节角度
        dq_all: (12,) 所有关节角速度

    返回：
        {leg_name: {'pos': p_foot_body (3,), 'vel': v_foot_body (3,)}}
    """
    results = {}
    for leg_name, idx in LEG_JOINT_MAP.items():
        q_leg = q_all[idx]
        dq_leg = dq_all[idx]

        p = forward_kinematics(q_leg, leg_name)
        J = compute_jacobian(q_leg, leg_name)
        v = foot_velocity(J, dq_leg)

        results[leg_name] = {'pos': p, 'vel': v}

    return results


def foot_velocity_in_world(R_body: np.ndarray,
                           p_foot_body: np.ndarray,
                           v_foot_body: np.ndarray,
                           omega_body: np.ndarray) -> np.ndarray:
    """
    将足端速度转换到世界坐标系。

    足端在世界系下的速度：
      v_foot_world = R_body · (v_foot_body + omega_body × p_foot_body)

    其中：
      - v_foot_body: 关节运动引起的足端相对速度（J · dq）
      - omega_body × p_foot_body: 机身旋转引起的足端线速度
      - R_body: 机身在世界系下的姿态

    参数：
        R_body: (3, 3) 机身在世界系下的旋转矩阵
        p_foot_body: (3,) 足端在机身系下的位置
        v_foot_body: (3,) 足端在机身系下的相对速度
        omega_body: (3,) 机身角速度（机身坐标系）

    返回：
        v_foot_world: (3,) 足端在世界系下的速度 (m/s)
    """
    # 机身旋转引起的足端线速度（机身坐标系）
    v_rot = np.cross(omega_body, p_foot_body)
    # 总足端速度（机身坐标系）
    v_total_body = v_foot_body + v_rot
    # 转到世界坐标系
    v_total_world = R_body @ v_total_body
    return v_total_world


def extract_frame_data(df_row) -> Tuple[np.ndarray, np.ndarray]:
    """
    从 DataFrame 的一行中提取关节角度和角速度。

    参数：
        df_row: DataFrame 的一行（pandas Series）

    返回：
        (q_all, dq_all): 均为 (12,) ndarray
    """
    q_all = df_row[JOINT_Q_COLS].values.astype(float)
    dq_all = df_row[JOINT_DQ_COLS].values.astype(float)
    return q_all, dq_all


def leg_odometry_velocity(
    q_all: np.ndarray,
    dq_all: np.ndarray,
    R_body: np.ndarray,
    omega_body: np.ndarray,
    contact_confidence: Dict[str, float],
) -> Tuple[np.ndarray, np.ndarray]:
    """
    腿式里程计：利用触地腿的零速约束反解机身速度。

    核心原理（ZUPT）：
      触地足端在世界系下的速度 ≈ 0
      =>  R_body · (J·dq + ω_body × p_foot_body) + v_body_world = 0
      =>  v_body_world = -R_body · (J·dq + ω_body × p_foot_body)

    对每条触地腿独立估计，然后按触地置信度加权融合。

    参数：
        q_all: (12,) 所有关节角度
        dq_all: (12,) 所有关节角速度
        R_body: (3, 3) 机身在世界系下的旋转矩阵
        omega_body: (3,) 机身角速度（机身坐标系）
        contact_confidence: {leg_name: 触地置信度 (0~1)}，如 {'FL': 0.95, 'FR': 0.02, ...}

    返回：
        (v_body_world, weight_sum):
            v_body_world: (3,) 机身在世界系下的线速度估计 (m/s)
            weight_sum: 总权重（用于协方差缩放）
    """
    v_est = np.zeros(3)       # 加权速度累加器
    weight_sum = 0.0          # 总权重

    for leg_name, idx in LEG_JOINT_MAP.items():
        w = contact_confidence.get(leg_name, 0.0)
        if w < 1e-6:
            continue  # 跳过几乎不触地的腿

        q_leg = q_all[idx]
        dq_leg = dq_all[idx]

        # 足端位置（机身系）
        p_foot = forward_kinematics(q_leg, leg_name)

        # 足端相对速度（机身系）：J · dq
        J = compute_jacobian(q_leg, leg_name)
        v_rel_body = foot_velocity(J, dq_leg)

        # 机身旋转引起的足端线速度（机身系）：ω × p
        v_rot_body = np.cross(omega_body, p_foot)

        # 触地腿零速约束反解机身速度（世界系）
        # v_foot_world = R·(v_rel_body + v_rot_body) + v_body_world = 0
        # => v_body_world = -R·(v_rel_body + v_rot_body)
        v_body_world_i = -R_body @ (v_rel_body + v_rot_body)

        v_est += w * v_body_world_i
        weight_sum += w

    if weight_sum > 1e-6:
        v_est /= weight_sum
    else:
        # 无有效触地腿，返回零速度（或可改为返回 None 让调用方处理）
        v_est[:] = 0.0

    return v_est, weight_sum


# ============================================================
# 自检（python3 -m ... .kinematics 运行）
# 只保留能真正验证正确性的检查，不放只验算术的假数据用例
# ============================================================
if __name__ == "__main__":
    print("=" * 60)
    print("  正运动学自检")
    print("=" * 60)

    # [1] 几何 sanity：站立直腿（q=0）足端应在髋正下方，竖直长度=大腿+小腿
    print("\n[1] 站立直腿几何")
    p = forward_kinematics(np.zeros(3), 'FL')
    expected_z = HIP_OFFSET['FL'][2] - (L_THIGH + L_SHANK)
    print(f"  足端 = {np.round(p, 4)}")
    assert abs(p[0] - HIP_OFFSET['FL'][0]) < 1e-9, "直腿时足端X应等于髋X偏移"
    assert abs(p[2] - expected_z) < 1e-9, "直腿时足端Z应=髋Z-(大腿+小腿)"
    print(f"  ✓ X对齐髋偏移, Z={p[2]:.4f}=-(L_THIGH+L_SHANK)")

    # [2] 雅可比数值校验：解析 J 必须与有限差分一致（这是能抓住建模错误的关键检查）
    print("\n[2] 雅可比 vs 有限差分")
    max_err = 0.0
    for q_test in [np.zeros(3), np.array([0.05, -0.4, 0.8]),
                   np.array([0.3, 0.6, -1.2]), np.array([-0.2, 0.5, -1.0])]:
        for leg in ['FL', 'FR', 'HL', 'HR']:
            J_ana = compute_jacobian(q_test, leg)
            J_num = np.zeros((3, 3))
            eps = 1e-7
            for k in range(3):
                qp, qm = q_test.copy(), q_test.copy()
                qp[k] += eps; qm[k] -= eps
                J_num[:, k] = (forward_kinematics(qp, leg)
                               - forward_kinematics(qm, leg)) / (2 * eps)
            max_err = max(max_err, np.max(np.abs(J_ana - J_num)))
    print(f"  最大误差 = {max_err:.2e}")
    assert max_err < 1e-5, "雅可比与有限差分不一致，运动学建模有误！"
    print("  ✓ 解析雅可比正确")

    # [3] 物理方向：只动 hipPitch 必须产生前后(X)位移；只动 hipRoll 不应改变 X
    print("\n[3] 关节-方向对应")
    Jp = compute_jacobian(np.array([0.0, 0.5, -1.0]), 'FL')
    assert abs(Jp[0, 1]) > 1e-3, "hipPitch 必须能改变足端X(否则无法迈步)"
    assert abs(Jp[0, 0]) < 1e-9, "hipRoll 不应改变足端X"
    print(f"  ✓ ∂X/∂pitch={Jp[0,1]:+.3f}(非零), ∂X/∂roll={Jp[0,0]:+.1f}(零)")

    # [4] 腿式里程计边界：全摆动(无触地)→ 速度归零、权重0
    print("\n[4] 腿式里程计无触地边界")
    v, w = leg_odometry_velocity(np.zeros(12), np.zeros(12), np.eye(3),
                                 np.zeros(3), {'FL': 0, 'FR': 0, 'HL': 0, 'HR': 0})
    assert np.allclose(v, 0) and w == 0.0
    print(f"  ✓ 无触地腿时 v={v}, weight={w}")

    print("\n[完成] 全部自检通过")