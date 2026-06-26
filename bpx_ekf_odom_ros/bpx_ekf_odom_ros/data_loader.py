#!/usr/bin/env python3
"""
data_loader.py - CSV 数据加载与校验

功能：
  1. load_csv(filepath) → pd.DataFrame
     读取 CSV 文件，自动解析时间戳为相对时间（秒）
  2. validate_data(df) → dict
     检查缺失值、数据类型、数据范围
  3. get_static_segment(df) → pd.DataFrame
     提取静态段（cmd_phase==2）用于零偏标定
  4. get_phase_segments(df) → list
     提取各运动阶段（cmd_phase 0~9）的索引范围
  5. get_data_by_leg(df, leg_name) → dict
     按腿名提取关节角度/角速度/力矩

依赖：
  pip install pandas numpy
"""

import numpy as np
import pandas as pd
from pathlib import Path
from typing import Optional, Dict, List, Tuple


# ============================================================
# 常量定义
# ============================================================

# 关节名称（BPX 四足机器人，每条腿 3 个关节）
# SDK motion_types.h 实际顺序: FL(0-2), FR(3-5), HL(6-8), HR(9-11)
JOINT_NAMES = [
    "FL_hip", "FL_thigh", "FL_calf",    # 左前腿 (0-2)
    "FR_hip", "FR_thigh", "FR_calf",    # 右前腿 (3-5)
    "HL_hip", "HL_thigh", "HL_calf",    # 左后腿 (6-8)
    "HR_hip", "HR_thigh", "HR_calf",    # 右后腿 (9-11)
]

# 腿名到关节索引的映射
LEG_JOINT_MAP = {
    'FL': [0, 1, 2],   # FLHipRoll, FLHipPitch, FLKnee
    'FR': [3, 4, 5],   # FRHipRoll, FRHipPitch, FRKnee
    'HL': [6, 7, 8],   # HLHipRoll, HLHipPitch, HLKnee
    'HR': [9, 10, 11], # HRHipRoll, HRHipPitch, HRKnee
}

# 运动阶段名称
PHASE_NAMES = {
    0: "标零", 1: "起立", 2: "站立不动",
    3: "前进", 4: "左转", 5: "后退",
    6: "右转", 7: "停止", 8: "坐下", 9: "完成"
}

# 数据合理性检查范围
IMU_ACC_RANGE = (-60.0, 60.0)      # m/s²
IMU_OMEGA_RANGE = (-35.0, 35.0)    # rad/s
JOINT_Q_RANGE = (-3.5, 3.5)        # rad
JOINT_DQ_RANGE = (-20.0, 20.0)     # rad/s
JOINT_TAU_RANGE = (-30.0, 30.0)    # Nm

# 缺失值阈值（超过此比例报警）
MISSING_RATIO_THRESHOLD = 0.1  # 10%


class DataLoader:
    """CSV 数据加载器"""

    def __init__(self):
        self._raw_df = None
        self._df = None

    # ----------------------------------------------------------
    # 1. 加载 CSV
    # ----------------------------------------------------------
    def load_csv(self, filepath: str) -> pd.DataFrame:
        """
        加载 CSV 数据文件

        参数：
            filepath: CSV 文件路径

        返回：
            pd.DataFrame，包含所有传感器数据
        """
        filepath = str(filepath)
        print(f"[DataLoader] 读取数据文件: {filepath}")

        if not Path(filepath).exists():
            raise FileNotFoundError(f"文件不存在: {filepath}")

        df = pd.read_csv(filepath)
        self._raw_df = df.copy()

        # 【列名规范化】不同录制版本的关节列名带后缀（如 q_00_FL_hip_roll），
        # 但脚本各处用 q_00 / dq_00 / tau_00。这里统一截断为标准短名，
        # 否则带后缀的文件所有关节相关功能都会静默失效。
        import re
        rename_map = {}
        for c in df.columns:
            m = re.match(r'^((?:q|dq|tau)_\d{2})(?:_.*)?$', c)
            if m and m.group(1) != c:
                rename_map[c] = m.group(1)
        if rename_map:
            df = df.rename(columns=rename_map)
            print(f"[DataLoader] 列名规范化: {len(rename_map)} 个关节列已截断后缀")

        print(f"[DataLoader] 原始帧数: {len(df)}")
        print(f"[DataLoader] 列数: {len(df.columns)}")

        # 检查必要列是否存在
        required_cols = ['timestamp_ms', 'imu_acc_x', 'imu_omega_x']
        missing = [c for c in required_cols if c not in df.columns]
        if missing:
            raise ValueError(f"缺少必要列: {missing}")

        # 【空文件保护】部分录制文件只有表头、没有数据行
        if len(df) == 0:
            raise ValueError(f"文件无数据行（仅表头）: {filepath}")

        # 将 timestamp_ms 转换为相对时间（秒）
        #
        # 【2026-06 数据特征更新】新录制的 timestamp_ms 已是高频严格单调序列
        # （每帧唯一、dt≈10ms≈96Hz），不再是早期“粗分辨率 50ms/20Hz、同一时间戳
        # 对应多帧”的情况。因此这里不需要任何时间戳重建，直接换算秒即可。
        # 真正需要关注的“值层面重采样重复帧”交由 preprocessor 处理，不在加载层动。
        if 'timestamp_s' not in df.columns:
            ts = pd.to_numeric(df['timestamp_ms'], errors='coerce')
            if ts.notna().sum() == 0:
                # 【NaN 时间戳保护】整列时间戳无效（如某些录制 timestamp 全空）。
                # 退化方案：若有 seq 列按采样序号、否则按行号生成时间，
                # 频率未知时假设标称采样率（这里按 100Hz=10ms）。
                NOMINAL_DT_MS = 10.0
                if 'seq' in df.columns and pd.to_numeric(
                        df['seq'], errors='coerce').notna().any():
                    seq = pd.to_numeric(df['seq'], errors='coerce')
                    seq = seq.fillna(method='ffill').fillna(0)
                    df['timestamp_s'] = (seq - seq.iloc[0]) * NOMINAL_DT_MS / 1000.0
                    print("[DataLoader] ⚠️ timestamp_ms 全无效，已按 seq 列"
                          f"生成相对时间（假设 {1000/NOMINAL_DT_MS:.0f}Hz）")
                else:
                    df['timestamp_s'] = np.arange(len(df)) * NOMINAL_DT_MS / 1000.0
                    print("[DataLoader] ⚠️ timestamp_ms 全无效且无 seq，已按行号"
                          f"生成相对时间（假设 {1000/NOMINAL_DT_MS:.0f}Hz）")
            else:
                # 用首个有效时间戳作为 t0；个别 NaN 由后续插值处理
                t0 = ts.dropna().iloc[0]
                df['timestamp_ms'] = ts
                df['timestamp_s'] = (ts - t0) / 1000.0

        # 【时间戳体检】快速判断新数据是否为“已单调高频”这一常态，
        # 顺便估计真实采样率，给后续步骤一个明确预期。
        ts_valid = pd.to_numeric(df['timestamp_ms'], errors='coerce').dropna()
        if len(ts_valid) > 1:
            dts = np.diff(ts_valid.values)
            n_nonpos = int((dts <= 0).sum())
            span_s = (ts_valid.iloc[-1] - ts_valid.iloc[0]) / 1000.0
            fs = (len(ts_valid) - 1) / span_s if span_s > 0 else float('nan')
            print(f"[DataLoader] 时间戳: {'严格单调' if n_nonpos == 0 else f'⚠️ {n_nonpos} 处非递增'}"
                  f", 录制采样率≈{fs:.1f}Hz, 中位 dt={np.median(dts):.1f}ms")

        # 【cmd_phase 提示】静态段标定/相位分段都依赖该列，缺失则提前告警
        if 'cmd_phase' not in df.columns:
            print("[DataLoader] ⚠️ 缺少 'cmd_phase' 列，静态段提取/相位分段将不可用")

        print(f"[DataLoader] 时间范围: {df['timestamp_s'].min():.2f} ~ "
              f"{df['timestamp_s'].max():.2f} 秒")
        print(f"[DataLoader] 列名: {list(df.columns)}")

        self._df = df
        return df

    # ----------------------------------------------------------
    # 2. 数据校验
    # ----------------------------------------------------------
    def validate_data(self, df: Optional[pd.DataFrame] = None,
                      exclude_phase_zero: bool = True) -> Dict:
        """
        检查数据质量

        参数：
            df: 要检查的 DataFrame（默认使用已加载的数据）
            exclude_phase_zero: 若为 True，检查关节角度范围时排除 cmd_phase==0
                                （标零阶段）的帧，因为这些帧的 q 值可能是垃圾数据

        返回：
            dict，包含检查结果
        """
        if df is None:
            df = self._df
        if df is None:
            raise ValueError("请先调用 load_csv() 加载数据")

        results = {
            'total_frames': len(df),
            'missing_ratio': {},
            'range_errors': {},
            'warnings': [],
            'errors': [],
        }

        # 2a. 检查缺失值
        missing_ratio = df.isnull().mean()
        for col, ratio in missing_ratio.items():
            if ratio > 0:
                results['missing_ratio'][col] = float(ratio)
                if ratio > MISSING_RATIO_THRESHOLD:
                    results['warnings'].append(
                        f"列 '{col}' 缺失率 {ratio:.1%} > {MISSING_RATIO_THRESHOLD:.0%}"
                    )

        # 2b. 检查 IMU 数据范围
        imu_acc_cols = ['imu_acc_x', 'imu_acc_y', 'imu_acc_z']
        for col in imu_acc_cols:
            if col in df.columns:
                out_of_range = df[col].dropna()
                out_of_range = out_of_range[
                    (out_of_range < IMU_ACC_RANGE[0]) |
                    (out_of_range > IMU_ACC_RANGE[1])
                ]
                if len(out_of_range) > 0:
                    results['range_errors'][col] = {
                        'count': len(out_of_range),
                        'min': float(out_of_range.min()),
                        'max': float(out_of_range.max()),
                    }
                    results['warnings'].append(
                        f"IMU 加速度 '{col}' 有 {len(out_of_range)} 帧超出范围 "
                        f"{IMU_ACC_RANGE}"
                    )

        imu_omega_cols = ['imu_omega_x', 'imu_omega_y', 'imu_omega_z']
        for col in imu_omega_cols:
            if col in df.columns:
                out_of_range = df[col].dropna()
                out_of_range = out_of_range[
                    (out_of_range < IMU_OMEGA_RANGE[0]) |
                    (out_of_range > IMU_OMEGA_RANGE[1])
                ]
                if len(out_of_range) > 0:
                    results['range_errors'][col] = {
                        'count': len(out_of_range),
                        'min': float(out_of_range.min()),
                        'max': float(out_of_range.max()),
                    }
                    results['warnings'].append(
                        f"IMU 角速度 '{col}' 有 {len(out_of_range)} 帧超出范围 "
                        f"{IMU_OMEGA_RANGE}"
                    )

        # 2c. 检查关节角度范围
        q_cols = [f"q_{i:02d}" for i in range(12)]
        # 如果启用排除标零阶段，且数据包含 cmd_phase 列
        phase_col = 'cmd_phase'
        has_phase = exclude_phase_zero and phase_col in df.columns
        if has_phase:
            n_phase_zero = int((df[phase_col] == 0).sum())
            if n_phase_zero > 0:
                print(f"[DataLoader] 关节角度范围检查: 排除 {n_phase_zero} 帧 "
                      f"cmd_phase==0（标零阶段）")
        for col in q_cols:
            if col in df.columns:
                data = df[col].dropna()
                if has_phase:
                    # 只检查非标零阶段的帧
                    mask = df[phase_col] != 0
                    data = df.loc[mask, col].dropna()
                out_of_range = data[
                    (data < JOINT_Q_RANGE[0]) |
                    (data > JOINT_Q_RANGE[1])
                ]
                if len(out_of_range) > 0:
                    results['range_errors'][col] = {
                        'count': len(out_of_range),
                        'min': float(out_of_range.min()),
                        'max': float(out_of_range.max()),
                    }
                    results['warnings'].append(
                        f"关节角度 '{col}' 有 {len(out_of_range)} 帧超出范围 "
                        f"{JOINT_Q_RANGE}"
                    )

        # 2d. 检查重复时间戳
        if 'timestamp_ms' in df.columns:
            dup_count = df['timestamp_ms'].duplicated().sum()
            if dup_count > 0:
                results['duplicate_timestamps'] = int(dup_count)
                results['warnings'].append(
                    f"发现 {dup_count} 个重复时间戳"
                )

        # 2e. 检查时间间隔是否均匀
        if 'timestamp_s' in df.columns:
            dt = df['timestamp_s'].diff().dropna()
            dt_mean = dt.mean()
            dt_std = dt.std()
            dt_max = dt.max()
            dt_min = dt.min()
            results['dt_stats'] = {
                'mean': float(dt_mean),
                'std': float(dt_std),
                'min': float(dt_min),
                'max': float(dt_max),
            }
            if dt_std > dt_mean * 0.5:
                results['warnings'].append(
                    f"时间间隔不均匀: 均值={dt_mean:.4f}s, "
                    f"标准差={dt_std:.4f}s, 范围=[{dt_min:.4f}, {dt_max:.4f}]s"
                )

        # 打印摘要
        print(f"\n[DataLoader] 数据校验结果:")
        print(f"  总帧数: {results['total_frames']}")
        print(f"  缺失列数: {len(results['missing_ratio'])}")
        print(f"  范围异常列数: {len(results['range_errors'])}")
        print(f"  警告数: {len(results['warnings'])}")
        for w in results['warnings']:
            print(f"    ⚠️ {w}")

        return results

    # ----------------------------------------------------------
    # 3. 提取静态段
    # ----------------------------------------------------------
    def get_static_segment(self, df: Optional[pd.DataFrame] = None,
                           phase: int = 2) -> pd.DataFrame:
        """
        提取静态段数据（用于 IMU 零偏标定）

        参数：
            df: DataFrame（默认使用已加载的数据）
            phase: 静态段对应的 cmd_phase 值（默认 2=站立不动）

        返回：
            pd.DataFrame，仅包含静态段的数据
        """
        if df is None:
            df = self._df
        if df is None:
            raise ValueError("请先调用 load_csv() 加载数据")

        if 'cmd_phase' not in df.columns:
            raise ValueError("数据中缺少 'cmd_phase' 列")

        static_df = df[df['cmd_phase'] == phase].copy()
        print(f"[DataLoader] 静态段 (cmd_phase=={phase}): "
              f"{len(static_df)} 帧")

        if len(static_df) == 0:
            print(f"  ⚠️ 未找到 cmd_phase=={phase} 的数据")

        return static_df

    # ----------------------------------------------------------
    # 4. 提取运动阶段
    # ----------------------------------------------------------
    def get_phase_segments(self, df: Optional[pd.DataFrame] = None) -> List[Dict]:
        """
        提取各运动阶段的起止索引

        参数：
            df: DataFrame（默认使用已加载的数据）

        返回：
            list of dict，每个元素包含 phase, name, start, end, count
        """
        if df is None:
            df = self._df
        if df is None:
            raise ValueError("请先调用 load_csv() 加载数据")

        if 'cmd_phase' not in df.columns:
            raise ValueError("数据中缺少 'cmd_phase' 列")

        phases = df['cmd_phase'].values
        unique_phases = np.unique(phases)
        segments = []

        for phase in sorted(unique_phases):
            if np.isnan(phase):
                continue
            phase = int(phase)
            mask = phases == phase
            indices = np.where(mask)[0]
            if len(indices) > 0:
                segments.append({
                    'phase': phase,
                    'name': PHASE_NAMES.get(phase, f"未知({phase})"),
                    'start': int(indices[0]),
                    'end': int(indices[-1]),
                    'count': len(indices),
                })

        print(f"[DataLoader] 共检测到 {len(segments)} 个运动阶段:")
        for seg in segments:
            print(f"  {seg['name']}: 帧 {seg['start']} ~ {seg['end']} "
                  f"({seg['count']}帧)")

        return segments

    # ----------------------------------------------------------
    # 5. 按腿提取数据
    # ----------------------------------------------------------
    def get_data_by_leg(self, df: Optional[pd.DataFrame] = None,
                        leg_name: str = 'FL') -> Dict:
        """
        按腿名提取关节角度/角速度/力矩

        参数：
            df: DataFrame（默认使用已加载的数据）
            leg_name: 腿名，'FL', 'FR', 'HL', 'HR'

        返回：
            dict，包含 q, dq, tau 三个 (N, 3) 的 numpy 数组
        """
        if df is None:
            df = self._df
        if df is None:
            raise ValueError("请先调用 load_csv() 加载数据")

        leg_name = leg_name.upper()
        if leg_name not in LEG_JOINT_MAP:
            raise ValueError(f"无效的腿名: {leg_name}，可选: {list(LEG_JOINT_MAP.keys())}")

        joint_indices = LEG_JOINT_MAP[leg_name]

        q_cols = [f"q_{i:02d}" for i in joint_indices]
        dq_cols = [f"dq_{i:02d}" for i in joint_indices]
        tau_cols = [f"tau_{i:02d}" for i in joint_indices]

        result = {}
        if all(c in df.columns for c in q_cols):
            result['q'] = df[q_cols].values  # (N, 3)
        if all(c in df.columns for c in dq_cols):
            result['dq'] = df[dq_cols].values  # (N, 3)
        if all(c in df.columns for c in tau_cols):
            result['tau'] = df[tau_cols].values  # (N, 3)

        return result

    # ----------------------------------------------------------
    # 6. 获取所有腿的数据（一帧）
    # ----------------------------------------------------------
    def get_frame_data(self, df: Optional[pd.DataFrame] = None,
                       idx: int = 0) -> Dict:
        """
        获取第 idx 帧的所有传感器数据

        参数：
            df: DataFrame（默认使用已加载的数据）
            idx: 帧索引

        返回：
            dict，包含该帧的所有传感器数据
        """
        if df is None:
            df = self._df
        if df is None:
            raise ValueError("请先调用 load_csv() 加载数据")

        if idx < 0 or idx >= len(df):
            raise IndexError(f"帧索引 {idx} 超出范围 [0, {len(df)-1}]")

        row = df.iloc[idx]
        data = {}

        # 时间
        data['t'] = float(row['timestamp_s'])

        # IMU
        for col in ['imu_acc_x', 'imu_acc_y', 'imu_acc_z',
                     'imu_omega_x', 'imu_omega_y', 'imu_omega_z',
                     'imu_rpy_roll', 'imu_rpy_pitch', 'imu_rpy_yaw']:
            if col in df.columns:
                data[col] = float(row[col])

        # 关节
        for prefix in ['q', 'dq', 'tau']:
            cols = [f"{prefix}_{i:02d}" for i in range(12)]
            if all(c in df.columns for c in cols):
                data[prefix] = row[cols].values.astype(float)

        # 里程计参考
        for col in ['leg_odom_x', 'leg_odom_y', 'leg_odom_z',
                     'body_vel_x', 'body_vel_y', 'body_vel_z']:
            if col in df.columns:
                data[col] = float(row[col])

        # 状态
        for col in ['cmd_phase', 'motion_state', 'gait']:
            if col in df.columns:
                # pd.isna 对字符串/None 也安全，np.isnan 遇到非数值会抛错
                try:
                    data[col] = -1 if pd.isna(row[col]) else float(row[col])
                except (ValueError, TypeError):
                    data[col] = -1

        return data

    # ----------------------------------------------------------
    # 7. 属性
    # ----------------------------------------------------------
    @property
    def raw_data(self) -> Optional[pd.DataFrame]:
        """原始数据（未处理）"""
        return self._raw_df

    @property
    def data(self) -> Optional[pd.DataFrame]:
        """当前数据"""
        return self._df

    @property
    def num_frames(self) -> int:
        """帧数"""
        if self._df is None:
            return 0
        return len(self._df)

    @property
    def duration(self) -> float:
        """数据时长（秒）"""
        if self._df is None or 'timestamp_s' not in self._df.columns:
            return 0.0
        return float(self._df['timestamp_s'].max() - self._df['timestamp_s'].min())


# ============================================================
# 便捷函数（无需实例化）
# ============================================================

def load_csv(filepath: str) -> pd.DataFrame:
    """便捷函数：加载 CSV 文件"""
    loader = DataLoader()
    return loader.load_csv(filepath)


def get_static_segment(df: pd.DataFrame, phase: int = 2) -> pd.DataFrame:
    """便捷函数：提取静态段"""
    loader = DataLoader()
    return loader.get_static_segment(df, phase)


def get_phase_segments(df: pd.DataFrame) -> List[Dict]:
    """便捷函数：提取运动阶段"""
    loader = DataLoader()
    return loader.get_phase_segments(df)


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

    # 测试 DataLoader
    loader = DataLoader()
    df = loader.load_csv(csv_path)

    # 数据校验
    results = loader.validate_data(df)

    # 提取静态段
    static_df = loader.get_static_segment(df)
    print(f"\n静态段帧数: {len(static_df)}")

    # 提取运动阶段
    segments = loader.get_phase_segments(df)

    # 按腿提取数据
    for leg in ['FL', 'FR', 'HL', 'HR']:
        leg_data = loader.get_data_by_leg(df, leg)
        print(f"\n{leg} 腿数据形状:")
        for key, val in leg_data.items():
            print(f"  {key}: {val.shape}")

    print("\n[完成] DataLoader 测试通过！")
