#!/usr/bin/env python3
"""
preprocessor.py - 时间对齐与预处理

功能：
  1. deduplicate(df) → pd.DataFrame
     去除重复时间戳的行
  2. detect_initial_zeros(df) → int
     检测 dq/tau 全零的初始区域，返回有效数据起始索引
  3. estimate_imu_bias(static_df, drop_first_n=50) → dict
     从静态段估计 IMU 零偏（陀螺 b_g, 加速度 b_a）
  4. remove_imu_bias(df, bias) → pd.DataFrame
     对全数据去除 IMU 零偏
  5. unwrap_joint_angles(df) → pd.DataFrame
     对关节角度做 np.unwrap 防止 ±π 跳变
  6. interpolate_to_uniform(df, target_dt=0.005) → pd.DataFrame
     插值到均匀时间网格（默认 200Hz, dt=5ms）
  7. preprocess_pipeline(filepath) → pd.DataFrame
     完整预处理流水线：加载 → 去重 → 去零偏 → 去跳变 → 插值

依赖：
  pip install pandas numpy scipy
"""

import numpy as np
import pandas as pd
from pathlib import Path
from typing import Optional, Dict, Tuple

try:
    from .data_loader import DataLoader      # 作为包导入时
except ImportError:
    from data_loader import DataLoader        # 直接运行 python preprocessor.py 时


# ============================================================
# 1. 去重
# ============================================================
def deduplicate(df: pd.DataFrame,
                time_col: str = 'timestamp_ms',
                drop_value_repeats: bool = False) -> pd.DataFrame:
    """
    清理时间戳并标记传感器“值重复帧”（默认不删除，仅标记）。

    【2026-06 数据特征更新 —— 与旧版假设相反，务必理解】
        旧版假设：timestamp_ms 是粗分辨率（~50ms/20Hz），同一时间戳对应多帧
        **不同**的高频传感器数据，于是按时间戳细分重建时间。

        新数据实测：
          - timestamp_ms 已是 **高频严格单调** 序列（每帧唯一、dt≈10ms≈96Hz），
            根本不存在“同一时间戳多帧”，旧的时间戳重建逻辑已无用武之地；
          - 真正的现象在 **值层面**：底层传感器更新率只有 ~25Hz，却被 ~96Hz
            录制重采样，导致约 74% 的行与前一帧传感器值**完全相同**（仅 timestamp
            在变）。这些是“传感器静默期的重采样帧”，不是噪声、也不是坏数据。

    为什么默认不删这些重复帧（对 EKF 更友好）：
        - 删掉它们 = 丢失“这段时间没有新观测”的真实信息，后续插值会用直线
          编造出传感器从未输出过的中间值，等于向 EKF 注入虚假观测；
        - 全保留但把每帧都当独立观测，又会让 EKF 重复消化同一观测、低估测量
          噪声、过度自信。
        正确折中：**保留全部帧维持真实时间轴**，仅用 `is_new_sample` 标记真正
        的更新时刻，让 EKF 只在新观测帧执行更新步、其余帧只做 IMU 预测积分。

    参数：
        df: 输入 DataFrame
        time_col: 时间戳列名
        drop_value_repeats: 若为 True，则真的删除值重复帧（降采样到 ~25Hz）。
                            默认 False（保留 + 标记），仅在你明确需要时开启。

    返回：
        新增 'is_new_sample' 列、时间戳已校验的 DataFrame
    """
    df = df.copy().reset_index(drop=True)
    before = len(df)

    # ---- 1) 标记“值重复帧”：与上一帧所有传感器值是否完全相同 ----
    value_cols = [c for c in df.columns
                  if c not in (time_col, 'timestamp_s', 'seq', 'is_new_sample')]
    if value_cols:
        # 与前一行逐列比较；首帧恒为新样本
        same_as_prev = (df[value_cols] == df[value_cols].shift(1)).all(axis=1)
        same_as_prev.iloc[0] = False
        is_new = ~same_as_prev
    else:
        is_new = pd.Series(True, index=df.index)

    n_repeat = int((~is_new).sum())
    eff_fs_note = ""
    if time_col in df.columns and df[time_col].notna().sum() > 1:
        span_s = (pd.to_numeric(df[time_col], errors='coerce').iloc[-1]
                  - pd.to_numeric(df[time_col], errors='coerce').iloc[0]) / 1000.0
        if span_s > 0:
            eff_fs_note = (f"，真实更新率≈{is_new.sum()/span_s:.1f}Hz"
                           f"（录制率≈{before/span_s:.1f}Hz）")

    if drop_value_repeats:
        df = df[is_new.values].reset_index(drop=True)
        df['is_new_sample'] = True
        print(f"[Preprocessor] 去重(降采样模式): 删除 {n_repeat} 个值重复帧 "
              f"({before} → {len(df)}){eff_fs_note}")
    else:
        df['is_new_sample'] = is_new.values
        print(f"[Preprocessor] 去重(保留+标记模式): {n_repeat}/{before} 帧为值重复"
              f"（已标记 is_new_sample，未删除）{eff_fs_note}")

    # ---- 2) 时间戳安全网：仅在出现真异常（非递增/重复）时才修复 ----
    if time_col in df.columns and df[time_col].notna().any():
        t = pd.to_numeric(df[time_col], errors='coerce').values.astype(float)
        dt = np.diff(t)
        n_nonpos = int(np.sum(dt <= 0)) if len(dt) else 0
        if n_nonpos > 0:
            # 真出现非递增（乱序/重复时间戳）才动手：组内按行序均匀细分
            print(f"[Preprocessor] ⚠️ 检测到 {n_nonpos} 处非递增时间戳，执行重建")
            t_fixed = t.copy()
            i, n = 0, len(t)
            while i < n:
                j = i
                while j < n and t[j] == t[i]:
                    j += 1
                group = j - i
                if group > 1:
                    step = (t[j] - t[i]) if j < n else 10.0
                    if step <= 0:
                        step = 10.0
                    for k in range(group):
                        t_fixed[i + k] = t[i] + step * k / group
                i = j
            t = t_fixed
            df[time_col] = t
        # 统一（重新）生成相对秒
        t0 = t[~np.isnan(t)][0]
        df['timestamp_s'] = (t - t0) / 1000.0
        print(f"[Preprocessor] 时间轴: {len(df)} 帧, 时长 "
              f"{df['timestamp_s'].max():.2f}s, "
              f"{'严格单调' if n_nonpos == 0 else '已修复为单调'}")

    return df


# ============================================================
# 2. 检测初始零值区域
# ============================================================
def detect_initial_zeros(df: pd.DataFrame,
                         dq_cols: Optional[list] = None,
                         tau_cols: Optional[list] = None,
                         q_cols: Optional[list] = None,
                         threshold: float = 0.01,
                         min_consecutive: int = 5,
                         q_range: Tuple[float, float] = (-3.5, 3.5)) -> int:
    """
    检测初始无效数据区域，返回有效数据起始索引

    检测依据（两条件满足其一即视为无效帧）：
        1. dq/tau 全零（能量 < threshold）—— 机器人启动前关节未运动
        2. q 值超出物理范围（q_range）—— 标零阶段的垃圾帧

    原理：
        机器人启动前，关节角速度和力矩全为 0。
        此外，cmd_phase==0（标零）阶段的前几帧关节角度 q 可能为垃圾值，
        超出物理范围 [-3.5, 3.5] rad，这些帧也应被裁剪。

    参数：
        df: 输入 DataFrame
        dq_cols: 关节角速度列名列表（默认自动检测 dq_00~dq_11）
        tau_cols: 关节力矩列名列表（默认自动检测 tau_00~tau_11）
        q_cols: 关节角度列名列表（默认自动检测 q_00~q_11）
        threshold: 能量跳变阈值（绝对值之和超过此值视为有效数据开始）
        min_consecutive: 连续有效帧数要求（避免单帧噪声误判）
        q_range: 关节角度物理范围 (min, max)，超出此范围视为无效帧

    返回：
        int: 有效数据起始索引（0 表示未检测到无效区域）
    """
    if dq_cols is None:
        dq_cols = [f"dq_{i:02d}" for i in range(12)]
    if tau_cols is None:
        tau_cols = [f"tau_{i:02d}" for i in range(12)]
    if q_cols is None:
        q_cols = [f"q_{i:02d}" for i in range(12)]

    # 只使用数据中实际存在的列
    dq_cols = [c for c in dq_cols if c in df.columns]
    tau_cols = [c for c in tau_cols if c in df.columns]
    q_cols = [c for c in q_cols if c in df.columns]

    if not dq_cols and not tau_cols and not q_cols:
        print("[Preprocessor] 警告: 未找到 dq/tau/q 列，跳过初始无效帧检测")
        return 0

    n = len(df)
    invalid = np.zeros(n, dtype=bool)

    # 条件1：dq/tau 全零（能量 < threshold）
    if dq_cols or tau_cols:
        energy = np.zeros(n)
        if dq_cols:
            energy += np.sum(np.abs(df[dq_cols].values), axis=1)
        if tau_cols:
            energy += np.sum(np.abs(df[tau_cols].values), axis=1)
        invalid |= (energy < threshold)

    # 条件2：q 值超出物理范围
    if q_cols:
        q_lo, q_hi = q_range
        q_vals = df[q_cols].values
        q_out_of_range = np.any((q_vals < q_lo) | (q_vals > q_hi), axis=1)
        invalid |= q_out_of_range

    # 找到首次"连续 min_consecutive 帧"都有效（即 invalid==False）的位置
    valid_start = 0
    if min_consecutive <= 1:
        idx = np.argmax(~invalid)
        valid_start = int(idx) if idx > 0 else 0
    else:
        run = 0
        for i in range(n):
            run = run + 1 if not invalid[i] else 0
            if run >= min_consecutive:
                valid_start = i - min_consecutive + 1
                break

    # 统计无效帧原因
    n_zero = int(np.sum(energy < threshold)) if dq_cols or tau_cols else 0
    n_q_bad = int(np.sum(q_out_of_range)) if q_cols else 0

    if valid_start > 0:
        print(f"[Preprocessor] 初始无效帧检测: 前 {valid_start} 帧为无效数据 "
              f"(dq/tau 全零={n_zero}帧, q 越界={n_q_bad}帧)")
    else:
        print(f"[Preprocessor] 初始无效帧检测: 未检测到无效区域 "
              f"(dq/tau 全零={n_zero}帧, q 越界={n_q_bad}帧)")

    return valid_start


# ============================================================
# 3. IMU 零偏估计
# ============================================================
def estimate_imu_bias(static_df: pd.DataFrame,
                      drop_first_n: int = 50) -> Dict[str, np.ndarray]:
    """
    从静态段估计 IMU 零偏

    原理：
        机器人静止时：
        - 陀螺仪理论输出应为 0，均值即为零偏 b_g
        - 加速度计理论输出应为重力 g，均值 - [0,0,g] 即为零偏 b_a

    参数：
        static_df: 静态段 DataFrame（cmd_phase==2）
        drop_first_n: 丢弃前 N 帧瞬态数据（机器人刚进入静态段时的抖动）

    返回：
        dict: {
            'b_g': np.ndarray(3,),   # 陀螺零偏 (rad/s)
            'b_a': np.ndarray(3,),   # 加速度零偏 (m/s²)
            'g': float,              # 估计的重力加速度值
            'n_samples': int,        # 用于估计的样本数
        }
    """
    # 去重后静态段帧率降低，固定丢 50 帧可能过激进。
    # 限制最多丢弃前 30% 帧，并保证至少留 10 帧用于估计。
    drop_first_n = min(drop_first_n, int(len(static_df) * 0.3))
    if len(static_df) < drop_first_n + 10:
        print(f"[Preprocessor] 警告: 静态段帧数不足 "
              f"({len(static_df)} < {drop_first_n + 10})，"
              f"将使用全部数据")
        drop_first_n = max(0, len(static_df) - 10)

    # 丢弃前 N 帧瞬态数据
    if drop_first_n > 0:
        static_df = static_df.iloc[drop_first_n:].copy()
        print(f"[Preprocessor] IMU 零偏估计: 丢弃前 {drop_first_n} 帧瞬态数据")

    # 提取 IMU 数据
    omega_cols = ['imu_omega_x', 'imu_omega_y', 'imu_omega_z']
    acc_cols = ['imu_acc_x', 'imu_acc_y', 'imu_acc_z']

    if not all(c in static_df.columns for c in omega_cols):
        raise ValueError(f"静态段缺少陀螺仪列: {omega_cols}")
    if not all(c in static_df.columns for c in acc_cols):
        raise ValueError(f"静态段缺少加速度计列: {acc_cols}")

    omega = static_df[omega_cols].values  # (N, 3)
    acc = static_df[acc_cols].values      # (N, 3)

    # 陀螺零偏 = 均值（静止时理论为 0）
    b_g = np.mean(omega, axis=0)

    # 加速度零偏 b_a
    # 静止时：acc_meas = R_wb^T · g_world + b_a
    # 其中 g_world = [0, 0, -g]（世界系重力向下），R_wb 为 body→world 旋转。
    # 【关键】机身站立时有 roll/pitch 倾角，重力会分量到 body 系的 X/Y 轴，
    # 不能简单假设重力只在 Z 轴（acc_mean - [0,0,g] 是错误的）。
    acc_mean = np.mean(acc, axis=0)
    g_est = np.linalg.norm(acc_mean)  # 由测量模长估计重力大小

    rpy_cols = ['imu_rpy_roll', 'imu_rpy_pitch', 'imu_rpy_yaw']
    quat_cols = ['imu_quat_w', 'imu_quat_x', 'imu_quat_y', 'imu_quat_z']

    def _g_body_from_rpy():
        rpy = static_df[rpy_cols].values
        roll, pitch = np.mean(rpy[:, 0]), np.mean(rpy[:, 1])  # yaw 不影响重力投影
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        # 加速度计静止读数（指向上、模长 g）在 body 系：
        return np.array([-g_est * sp, g_est * sr * cp, g_est * cr * cp])

    g_body = None
    bias_method = None
    # 【首选 rpy】实测该 BPX 设备 rpy 与加速度计自洽（残差 < 0.05 m/s²），
    # 而 imu_quat_* 的坐标约定与加速度计不一致，投影会得到错误结果。
    if all(c in static_df.columns for c in rpy_cols):
        cand = _g_body_from_rpy()
        residual = np.linalg.norm(acc_mean - cand)
        # 一致性校验：水平残差应远小于 g；若过大说明姿态与加速度不自洽
        if residual < 0.3 * g_est:
            g_body = cand
            bias_method = 'rpy_compensated'
        else:
            print(f"[Preprocessor] ⚠️ rpy 投影残差过大 ({residual:.3f})，"
                  f"姿态与加速度不自洽")

    if g_body is None:
        # 姿态不可用或不自洽：b_a 与重力方向耦合无法分离，置零交给在线估计。
        print("[Preprocessor] ⚠️ 无可靠姿态，无法分离加速度零偏，"
              "b_a 置零（建议在线 EKF 估计）")
        b_a = np.zeros(3)
        bias_method = 'gyro_only'
    else:
        b_a = acc_mean - g_body
        print(f"[Preprocessor] 注：沿重力方向的 b_a 分量不可靠"
              f"（单姿态无法与重力大小分离），仅水平分量可信")

    result = {
        'b_g': b_g,
        'b_a': b_a,
        'g': g_est,
        'method': bias_method,
        'n_samples': len(static_df),
    }

    print(f"[Preprocessor] IMU 零偏估计结果 ({len(static_df)} 帧):")
    print(f"  陀螺零偏 b_g (rad/s): "
          f"[{b_g[0]:+.6f}, {b_g[1]:+.6f}, {b_g[2]:+.6f}]")
    print(f"  加速度零偏 b_a (m/s²): "
          f"[{b_a[0]:+.4f}, {b_a[1]:+.4f}, {b_a[2]:+.4f}]")
    print(f"  估计重力 g: {g_est:.4f} m/s²")

    return result


# ============================================================
# 4. 去除 IMU 零偏
# ============================================================
def remove_imu_bias(df: pd.DataFrame,
                    bias: Dict[str, np.ndarray]) -> pd.DataFrame:
    """
    对全数据去除 IMU 零偏

    参数：
        df: 输入 DataFrame
        bias: estimate_imu_bias() 返回的零偏字典

    返回：
        去除零偏后的 DataFrame（返回副本，不修改原 df）
    """
    df = df.copy()

    omega_cols = ['imu_omega_x', 'imu_omega_y', 'imu_omega_z']
    acc_cols = ['imu_acc_x', 'imu_acc_y', 'imu_acc_z']

    b_g = bias['b_g']
    b_a = bias['b_a']

    for i, col in enumerate(omega_cols):
        if col in df.columns:
            df[col] = df[col] - b_g[i]

    for i, col in enumerate(acc_cols):
        if col in df.columns:
            df[col] = df[col] - b_a[i]

    print(f"[Preprocessor] 已去除 IMU 零偏: "
          f"b_g={b_g}, b_a={b_a}")

    return df


# ============================================================
# 5. 关节角去跳变
# ============================================================
def unwrap_joint_angles(df: pd.DataFrame,
                        q_cols: Optional[list] = None,
                        phys_range: Tuple[float, float] = (-3.5, 3.5),
                        force: bool = False) -> pd.DataFrame:
    """
    谨慎处理关节角度的 ±2π 跳变。

    【重要】BPX 关节物理范围本就在 (-3.5, 3.5) rad 内，关节不会转整圈，
    因此正常数据**不应**出现真正的 ±2π wrap。数据里偶发的大跳变更可能是
    编码器丢包/噪声毛刺，对其无脑 np.unwrap 会把毛刺累加成持续偏移，
    把局部错误扩散到全程，反而破坏数据并使物理范围校验失效。

    本函数策略：
        - 默认只对"unwrap 后整体仍落在物理范围内"的关节应用 unwrap
          （即确实是被 wrap 的连续信号）；
        - 若 unwrap 后超出物理范围，判定为丢包毛刺，保持原值并告警，
          交由后续插值/滤波处理；
        - force=True 可强制对所有关节 unwrap（仅在确认存在真实 wrap 时使用）。

    参数：
        df: 输入 DataFrame
        q_cols: 关节角度列名列表（默认自动检测 q_00~q_11）
        phys_range: 关节物理角度范围 (rad)，用于判定是否为真实 wrap
        force: 强制 unwrap 所有关节

    返回：
        处理后的 DataFrame
    """
    if q_cols is None:
        q_cols = [f"q_{i:02d}" for i in range(12)]

    # 只处理数据中实际存在的列
    q_cols = [c for c in q_cols if c in df.columns]

    if not q_cols:
        print("[Preprocessor] 警告: 未找到关节角度列，跳过去跳变")
        return df

    df = df.copy()
    lo, hi = phys_range
    unwrapped_joints = []
    suspicious_joints = []

    for col in q_cols:
        original = df[col].values
        # 是否存在疑似 ±2π 跳变（相邻差 > π）
        if np.max(np.abs(np.diff(original))) <= np.pi:
            continue  # 无跳变，跳过

        candidate = np.unwrap(original)

        if force or (candidate.min() >= lo and candidate.max() <= hi):
            # unwrap 后仍在物理范围内 → 判定为真实 wrap，采用
            df[col] = candidate
            unwrapped_joints.append(col)
        else:
            # unwrap 后越界 → 判定为丢包毛刺，保持原值交由后续滤波
            suspicious_joints.append(col)

    if unwrapped_joints:
        print(f"[Preprocessor] 关节角去跳变: 对 {len(unwrapped_joints)} 个关节"
              f"执行 unwrap {unwrapped_joints}")
    if suspicious_joints:
        print(f"[Preprocessor] ⚠️ {len(suspicious_joints)} 个关节存在大跳变但"
              f"unwrap 后越界，疑似丢包/毛刺，已保持原值: {suspicious_joints}")
    if not unwrapped_joints and not suspicious_joints:
        print(f"[Preprocessor] 关节角去跳变: 未检测到跳变")

    return df


# ============================================================
# 6. 插值到均匀时间网格
# ============================================================
def interpolate_to_uniform(df: pd.DataFrame,
                           target_dt: float = 0.02,
                           time_col: str = 'timestamp_s',
                           discrete_cols: Optional[list] = None,
                           exclude_cols: Optional[list] = None) -> pd.DataFrame:
    """
    插值到均匀时间网格

    原理：
        原始数据时间间隔接近均匀（~96Hz, dt 10~12ms）但非严格等距，EKF 若要求
        固定 dt 可用本函数重采样到等间隔网格。

    【频率选择】真实传感器信息率仅 ~25Hz，默认目标改为 50Hz(dt=0.02s)，在
        “够 EKF 预测步细分”与“不过度虚假上采样”之间取平衡；旧的 200Hz 会把
        25Hz 的真实信息硬撑成 8 倍插值点，制造大量伪测量，不再作为默认。
        如确需更高频可显式传 target_dt。

    参数：
        df: 输入 DataFrame
        target_dt: 目标时间间隔（秒），默认 0.02 = 50Hz
        time_col: 时间列名
        discrete_cols: 离散列名列表（使用最近邻插值）
                      默认自动检测：cmd_phase, motion_state, gait, seq, is_new_sample
        exclude_cols: 排除列名列表（不参与插值）

    返回：
        插值后的 DataFrame，包含 'timestamp_s' 和 'dt' 列
    """
    if time_col not in df.columns:
        raise ValueError(f"时间列 '{time_col}' 不存在")

    # 默认离散列（is_new_sample 是布尔标记，必须最近邻，绝不能线性插值）
    if discrete_cols is None:
        discrete_cols = ['cmd_phase', 'motion_state', 'gait', 'seq',
                         'is_new_sample']

    # 默认排除列
    if exclude_cols is None:
        exclude_cols = ['timestamp_ms']

    # 获取原始时间
    t_raw = df[time_col].values
    t_start = t_raw[0]
    t_end = t_raw[-1]

    # 【保护】np.interp 要求 t_raw 严格递增，否则结果未定义。
    # 去重不彻底或时间戳乱序会导致静默错误，这里显式检查。
    dt_raw = np.diff(t_raw)
    if np.any(dt_raw <= 0):
        n_bad = int(np.sum(dt_raw <= 0))
        raise ValueError(
            f"时间列 '{time_col}' 非严格递增（{n_bad} 处非正间隔），"
            f"请先调用 deduplicate() 去重并确认时间戳已排序"
        )

    # 创建均匀时间网格
    t_uniform = np.arange(t_start, t_end, target_dt)

    # 如果最后一个点没被包含，补上
    if t_uniform[-1] < t_end - target_dt * 0.5:
        t_uniform = np.append(t_uniform, t_end)

    print(f"[Preprocessor] 插值到均匀网格:")
    print(f"  原始: {len(df)} 帧, "
          f"时间范围 [{t_start:.3f}, {t_end:.3f}]s, "
          f"平均 dt={np.mean(np.diff(t_raw)):.4f}s")
    print(f"  目标: {len(t_uniform)} 帧, "
          f"dt={target_dt}s ({1/target_dt:.0f}Hz)")

    # 准备插值结果
    result_dict = {time_col: t_uniform}
    result_dict['dt'] = np.full_like(t_uniform, target_dt)

    # 确定需要插值的列
    all_cols = [c for c in df.columns
                if c not in exclude_cols and c != time_col]

    continuous_cols = [c for c in all_cols if c not in discrete_cols]
    actual_discrete_cols = [c for c in discrete_cols if c in df.columns]

    # 6a. 连续变量：线性插值
    for col in continuous_cols:
        if col not in df.columns:
            continue
        raw_vals = df[col].values.astype(float)
        # 处理 NaN：用前后有效值填充
        valid_mask = ~np.isnan(raw_vals)
        if not valid_mask.any():
            # 整列全 NaN（如某些录制缺失该传感器），直接填 0 并跳过插值
            result_dict[col] = np.zeros_like(t_uniform)
            continue
        if not valid_mask.all():
            raw_vals = np.interp(
                t_raw, t_raw[valid_mask], raw_vals[valid_mask]
            )
        # 插值到均匀网格
        interp_vals = np.interp(t_uniform, t_raw, raw_vals)
        result_dict[col] = interp_vals

    # 6a-2. 四元数：线性插值会破坏单位范数，插值后重新归一化
    quat_cols = ['imu_quat_w', 'imu_quat_x', 'imu_quat_y', 'imu_quat_z']
    if all(c in result_dict for c in quat_cols):
        q = np.stack([result_dict[c] for c in quat_cols], axis=1)
        norms = np.linalg.norm(q, axis=1, keepdims=True)
        norms[norms == 0] = 1.0
        q = q / norms
        for i, c in enumerate(quat_cols):
            result_dict[c] = q[:, i]

    # 6b. 离散变量：最近邻插值
    # 用 scipy interp1d(kind='nearest') 替代手写 searchsorted，
    # 避免在等距/边界处出现 off-by-one 选错相邻帧（会导致相位边界±1帧错位）。
    if actual_discrete_cols:
        from scipy.interpolate import interp1d
        for col in actual_discrete_cols:
            raw_vals = df[col].values.astype(float)
            f_nn = interp1d(
                t_raw, raw_vals, kind='nearest',
                bounds_error=False,
                fill_value=(raw_vals[0], raw_vals[-1]),
            )
            interp_vals = f_nn(t_uniform)
            # 还原为整数类型（cmd_phase/motion_state/gait 等本质是整数标签）
            result_dict[col] = np.rint(interp_vals).astype(int)

    # 构建结果 DataFrame
    result_df = pd.DataFrame(result_dict)

    print(f"[Preprocessor] 插值完成: {len(result_df)} 帧")

    return result_df


# ============================================================
# 7. 完整预处理流水线
# ============================================================
def preprocess_pipeline(filepath: str,
                        target_dt: float = 0.02,
                        static_phase: int = 2,
                        drop_first_n: int = 50,
                        zero_threshold: float = 0.01,
                        interpolate: bool = False,
                        drop_value_repeats: bool = False,
                        verbose: bool = True) -> pd.DataFrame:
    """
    完整预处理流水线

    流程：
        加载 CSV → 去重(标记 is_new_sample) → 检测初始零值 → 提取静态段 →
        估计零偏 → 去除零偏 → 去跳变 → [可选] 插值到均匀网格

    参数：
        filepath: CSV 文件路径
        target_dt: 插值目标时间间隔（秒），默认 0.02 = 50Hz（仅 interpolate=True 时生效）
        static_phase: 静态段对应的 cmd_phase 值
        drop_first_n: 零偏估计时丢弃的前 N 帧
        zero_threshold: 初始零值检测阈值
        interpolate: 是否重采样到均匀网格。默认 False —— 新数据时间轴已近似均匀
                     (~96Hz)，保留原始帧 + 每帧真实 dt 对被动记录/变步长 EKF 更
                     忠实；若你的 EKF 要求严格固定 dt，置 True。
        drop_value_repeats: 是否删除 74% 的传感器值重复帧（降采样到真实 ~25Hz）。
                     默认 False（保留并用 is_new_sample 标记），见 deduplicate 说明。
        verbose: 是否打印详细信息

    返回：
        预处理后的 DataFrame（含 is_new_sample 列；interpolate=True 时另含 dt 列）
    """
    if verbose:
        print("=" * 60)
        print("  预处理流水线开始")
        print("=" * 60)

    # Step 1: 加载数据（空文件等异常优雅返回空 DataFrame，不中断批处理）
    loader = DataLoader()
    try:
        df = loader.load_csv(filepath)
    except ValueError as e:
        if verbose:
            print(f"[Pipeline] 跳过该文件: {e}")
        return pd.DataFrame()

    # Step 2: 去重（默认保留 + 标记 is_new_sample）
    df = deduplicate(df, drop_value_repeats=drop_value_repeats)

    # Step 3: 检测初始零值区域
    valid_start = detect_initial_zeros(df, threshold=zero_threshold)
    if valid_start > 0:
        df = df.iloc[valid_start:].reset_index(drop=True)
        if verbose:
            print(f"[Pipeline] 裁剪前 {valid_start} 帧无效数据")

    # Step 4: 提取静态段
    static_df = loader.get_static_segment(df, phase=static_phase)

    if len(static_df) > drop_first_n + 10:
        # Step 5: 估计 IMU 零偏
        bias = estimate_imu_bias(static_df, drop_first_n=drop_first_n)

        # Step 6: 去除零偏
        df = remove_imu_bias(df, bias)
    else:
        if verbose:
            print(f"[Pipeline] 警告: 静态段帧数不足 ({len(static_df)})，"
                  f"跳过零偏估计")
        bias = None

    # Step 7: 关节角去跳变
    df = unwrap_joint_angles(df)

    # Step 8: （可选）插值到均匀网格
    if interpolate:
        df_out = interpolate_to_uniform(df, target_dt=target_dt)
    else:
        # 保留原始时间轴，补一列每帧真实 dt 供变步长 EKF 使用
        df_out = df.reset_index(drop=True).copy()
        df_out['dt'] = df_out['timestamp_s'].diff().fillna(0.0)
        if verbose:
            print(f"[Pipeline] 保留原始时间轴（未插值），已附加每帧真实 dt 列")

    if verbose:
        print("=" * 60)
        rate = (1/target_dt if interpolate
                else (len(df_out)/df_out['timestamp_s'].max()
                      if df_out.get('timestamp_s') is not None
                      and len(df_out) and df_out['timestamp_s'].max() > 0 else float('nan')))
        print(f"  预处理完成: {len(df_out)} 帧, ≈{rate:.0f}Hz"
              f"{'（均匀网格）' if interpolate else '（原始时间轴）'}")
        print("=" * 60)

    return df_out


# ============================================================
# 便捷函数
# ============================================================

def preprocess_file(filepath: str, **kwargs) -> pd.DataFrame:
    """便捷函数：预处理单个 CSV 文件"""
    return preprocess_pipeline(filepath, **kwargs)


# ============================================================
# 测试入口
# ============================================================
if __name__ == "__main__":
    import sys

    # 查找最新的 CSV 文件
    project_dir = Path(__file__).parent.parent
    csv_files = sorted(project_dir.glob("record_*.csv"))

    # 如果根目录没找到，搜索 record_message2/ 子目录
    if not csv_files:
        csv_files = sorted(project_dir.glob("record_message2/record_*.csv"))

    if not csv_files:
        print("[错误] 未找到 CSV 数据文件！")
        sys.exit(1)

    # 使用最新的 CSV 文件
    csv_path = str(csv_files[-1])
    print(f"使用文件: {csv_path}")

    # 测试完整流水线
    df = preprocess_pipeline(csv_path)

    # 验证结果
    print(f"\n[验证] 预处理结果:")
    print(f"  帧数: {len(df)}")
    print(f"  列数: {len(df.columns)}")
    print(f"  时间范围: {df['timestamp_s'].min():.3f} ~ "
          f"{df['timestamp_s'].max():.3f} s")
    print(f"  dt 统计: 均值={df['dt'].mean():.6f}, "
          f"标准差={df['dt'].std():.6f}, "
          f"范围=[{df['dt'].min():.6f}, {df['dt'].max():.6f}]")

    # 检查 dt 是否均匀（原始录制 ~96Hz 天然有微小波动，用相对标准差判断）
    dt_mean = df['dt'].mean()
    dt_std = df['dt'].std()
    if dt_mean > 0 and dt_std / dt_mean < 0.1:
        print(f"  ✅ dt 近似均匀 (均值={dt_mean:.4f}s, "
              f"相对标准差={dt_std/dt_mean:.1%}, ≈{1/dt_mean:.0f}Hz)")
    else:
        print(f"  ⚠️ dt 不均匀 (均值={dt_mean:.6f}s, "
              f"标准差={dt_std:.6f}, 相对标准差={dt_std/dt_mean:.1%})")

    print("\n[完成] Preprocessor 测试通过！")
