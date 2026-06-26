"""
config.py - 所有可调参数（几何参数、EKF参数、阈值等）

所有参数集中管理，方便调参和对比实验。
"""

import numpy as np

# ============================================================
# 机器人几何参数（单位：米）
# 从 URDF 提取确认
# ============================================================

# 腿长参数
L_THIGH = 0.23        # 大腿长度（fl_knee_joint origin z=-0.23）
L_SHANK = 0.2427      # 小腿长度（fl_ankle_joint origin z=-0.2427）
HIP_WIDTH = 0.056     # 髋关节半宽（Y方向偏移）
BODY_HALF_L = 0.2432  # 前后髋半距（X方向偏移）

# 关节索引映射（与 motion_types.h 一致）
LEG_JOINT_MAP = {
    'FL': [0, 1, 2],   # FLHipRoll, FLHipPitch, FLKnee
    'FR': [3, 4, 5],   # FRHipRoll, FRHipPitch, FRKnee
    'HL': [6, 7, 8],   # HLHipRoll, HLHipPitch, HLKnee
    'HR': [9, 10, 11], # HRHipRoll, HRHipPitch, HRKnee
}

# 各腿髋关节在机身坐标系下的偏移
HIP_OFFSET = {
    'FL': np.array([ BODY_HALF_L,  HIP_WIDTH, 0.0]),
    'FR': np.array([ BODY_HALF_L, -HIP_WIDTH, 0.0]),
    'HL': np.array([-BODY_HALF_L,  HIP_WIDTH, 0.0]),
    'HR': np.array([-BODY_HALF_L, -HIP_WIDTH, 0.0]),
}

# 关节列名（用于从 DataFrame 提取数据）
JOINT_Q_COLS = [f"q_{i:02d}" for i in range(12)]
JOINT_DQ_COLS = [f"dq_{i:02d}" for i in range(12)]
JOINT_TAU_COLS = [f"tau_{i:02d}" for i in range(12)]

# IMU 列名
IMU_OMEGA_COLS = ['imu_omega_x', 'imu_omega_y', 'imu_omega_z']
IMU_ACC_COLS = ['imu_acc_x', 'imu_acc_y', 'imu_acc_z']
IMU_RPY_COLS = ['imu_rpy_roll', 'imu_rpy_pitch', 'imu_rpy_yaw']
IMU_QUAT_COLS = ['imu_quat_w', 'imu_quat_x', 'imu_quat_y', 'imu_quat_z']


# ============================================================
# 重力
# ============================================================

GRAVITY = 9.81  # m/s²，标准重力加速度
# 注：preprocessor 会从静态段估计实际 g 值（~9.87），
# 但 EKF 预测步中应使用标准值，零偏已由 preprocessor 去除


# ============================================================
# EKF 参数
# ============================================================

# --- 过程噪声协方差 Q（对角阵） ---
# 这些值控制 EKF 预测步的不确定性增长速率
# 2026-06-24 调优：增大 Q 使 EKF 更信任观测、减少 IMU 积分漂移
Q_ACC_NOISE = 0.05        # 加速度噪声 (m/s²)²（原 0.01，增大以允许观测修正）
Q_GYRO_NOISE = 0.01       # 角速度噪声 (rad/s)²（原 0.001，增大以允许姿态修正）
Q_BA_NOISE = 0.0001       # 加速度零偏随机游走 (m/s²)²/s
Q_BG_NOISE = 0.00001      # 陀螺零偏随机游走 (rad/s)²/s

# --- 观测噪声 R_base（基础值） ---
# 2026-06-24 调优：增大 R_base 使观测更保守，避免过度自信导致漂移
R_BASE_VEL = 0.01         # 足端速度观测噪声 (m/s)²（调至 0.01 使 NIS 进入 1.0~2.5 理想区间）

# --- 初始协方差 P0 ---
P0_POS = 0.01             # 位置初始不确定性 m²
P0_VEL = 0.01             # 速度初始不确定性 (m/s)²
P0_ATT = 0.001            # 姿态初始不确定性 rad²
P0_BG = 0.001             # 陀螺零偏初始不确定性 (rad/s)²
P0_BA = 0.1               # 加速度零偏初始不确定性 (m/s²)²


# ============================================================
# 触地检测参数
# ============================================================

# 力矩基线法
TAU_BASELINE_WINDOW = 50      # 静态段用于计算力矩基线的帧数
TAU_THRESHOLD_FACTOR = 2.0    # 力矩阈值 = max(1.0, 2×标准差)

# Sigmoid 参数
ALPHA_TORQUE = 5.0            # 力矩法 sigmoid 斜率
ALPHA_IMU = 3.0               # IMU 方差法 sigmoid 斜率
ALPHA_CONSISTENCY = 4.0       # 一致性法 sigmoid 斜率

# 触地置信度阈值（高于此值视为支撑腿）
CONTACT_THRESHOLD = 0.3

# 腿式里程计支撑腿硬门：喂进里程计前, 低于此置信度的腿权重清零, 并对
# 保留腿的权重做幂次锐化(避免摆动腿软权重污染加权平均)。
ODOM_SUPPORT_THRESHOLD = 0.5
ODOM_WEIGHT_POWER = 3.0

# 融合权重（足端速度法为唯一逐腿判据）
# 实测：力矩法在动态行走时被腿部惯性力污染，加入反而拉低相关性
#   (0.7速度+0.3力矩 → 相关0.41; 纯速度 → 0.78)，故权重清零。
#   如后续单独标定力矩法确认可靠，再调高 W_TORQUE。
# IMU 方差法仅作全局门控，不参与逐腿融合。
W_TORQUE = 0.0                # 力矩法权重（已禁用，待标定）
W_CONSISTENCY = 1.0           # 足端速度法权重（唯一主判据）

# 自适应观测噪声
EPSILON = 0.01                # 防止除零

# IMU 方差法滑动窗口大小（仅用于全局门控）
IMU_WINDOW_SIZE = 10


# ============================================================
# 文件路径
# ============================================================

# 默认 CSV 数据文件路径（相对于项目根目录）
DEFAULT_CSV_PATH = "record_message2/record_20260623_121431.csv"


# ============================================================
# 便捷函数
# ============================================================

def build_Q(dt: float) -> np.ndarray:
    """
    构建离散时间过程噪声协方差矩阵 Q_d (15×15)

    连续时间噪声：
      Q_c = diag(0_3x3, Q_v·I_3, Q_θ·I_3, Q_bg·I_3, Q_ba·I_3)

    离散化（一阶近似）：
      Q_d ≈ F · Q_c · F^T · dt
    简化：Q_d ≈ Q_c · dt（忽略 F 的交叉耦合项）

    参数：
        dt: 时间步长 (s)

    返回：
        Q_d: (15, 15) 离散过程噪声协方差
    """
    Q_c = np.zeros((15, 15))
    # 速度噪声（索引 3:6）
    Q_c[3:6, 3:6] = np.eye(3) * Q_ACC_NOISE
    # 姿态噪声（索引 6:9）
    Q_c[6:9, 6:9] = np.eye(3) * Q_GYRO_NOISE
    # 陀螺零偏噪声（索引 9:12）
    Q_c[9:12, 9:12] = np.eye(3) * Q_BG_NOISE
    # 加速度零偏噪声（索引 12:15）
    Q_c[12:15, 12:15] = np.eye(3) * Q_BA_NOISE

    return Q_c * dt


def build_P0() -> np.ndarray:
    """
    构建初始协方差矩阵 P0 (15×15)

    返回：
        P0: (15, 15) 初始协方差
    """
    P0 = np.zeros((15, 15))
    # 位置（索引 0:3）
    P0[0:3, 0:3] = np.eye(3) * P0_POS
    # 速度（索引 3:6）
    P0[3:6, 3:6] = np.eye(3) * P0_VEL
    # 姿态（索引 6:9）
    P0[6:9, 6:9] = np.eye(3) * P0_ATT
    # 陀螺零偏（索引 9:12）
    P0[9:12, 9:12] = np.eye(3) * P0_BG
    # 加速度零偏（索引 12:15）
    P0[12:15, 12:15] = np.eye(3) * P0_BA
    return P0