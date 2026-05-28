# bpx_driver — BPX 机器狗 ROS2 Humble 驱动功能包

基于 BPX SDK 的 ROS2 Humble 驱动，支持通过 `/cmd_vel` 话题控制机器狗运动，并实时发布机器狗状态。

## 目录

- [环境要求](#环境要求)
- [构建](#构建)
- [网络配置](#网络配置)
- [启动驱动](#启动驱动)
- [运动控制](#运动控制)
  - [站立与趴下](#站立与趴下)
  - [键盘遥控](#键盘遥控)
  - [程序化控制](#程序化控制)
- [话题](#话题)
- [服务](#服务)
- [参数](#参数)
- [常见问题](#常见问题)

---

## 环境要求

| 项目 | 要求 |
|------|------|
| 操作系统 | Ubuntu 22.04 |
| ROS2 版本 | Humble Hawksbill |
| 架构 | x86_64 或 aarch64 |
| 依赖 | `colcon`, `teleop_twist_keyboard`（遥控需要） |

安装遥控工具（如未安装）：

```bash
sudo apt install ros-humble-teleop-twist-keyboard
```

## 构建

```bash
cd ~/ros2_ws
colcon build --packages-select bpx_driver
source install/setup.bash
```

> 整个 `ros2_ws` 目录是自包含的，可以直接拷贝到其他满足环境要求的计算机上构建。

## 网络配置

1. **有线连接（推荐）**：通过网线直连机器狗，默认 IP 为 `10.21.20.1`
2. **无线连接**：连接到机器狗的 Wi-Fi 热点，默认 IP 为 `192.168.0.1`

连接前请先确认网络连通：

```bash
# 有线
ping 10.21.20.1

# 无线
ping 192.168.0.1
```

如果使用有线连接，需修改参数文件中的 IP 地址，见[参数](#参数)章节。

## 启动驱动

### 方式一：launch 启动（推荐）

```bash
# 如果端口被占用 
pkill -f ros

source install/setup.bash
ros2 launch bpx_driver bpx_driver.launch.py
```

### 方式二：ros2 run 启动

```bash
source install/setup.bash
ros2 run bpx_driver bpx_driver_node
```

启动成功后终端显示：

```
[INFO] [bpx_driver_node]: Connected to BPX robot at 192.168.0.1
[INFO] [bpx_driver_node]: BPX driver node started
```

### 指定 IP 启动

```bash
# 有线连接
ros2 launch bpx_driver bpx_driver.launch.py robot_ip:="10.21.20.1"
```

---

## 运动控制

### 站立与趴下

机器狗默认处于趴下状态，**必须先站立才能接收速度指令**。

```bash
# 站起
ros2 service call /bpx/stand_up std_srvs/srv/Trigger

# 趴下（自动停止运动）
ros2 service call /bpx/sit_down std_srvs/srv/Trigger

# 阻尼模式（关节放松，手动摆位）
ros2 service call /bpx/damping std_srvs/srv/Trigger

# 直立待机
ros2 service call /bpx/upright std_srvs/srv/Trigger
```

### 键盘遥控

**第一步**：启动驱动（终端 1）

```bash
source install/setup.bash
ros2 launch bpx_driver bpx_driver.launch.py
```

**第二步**：站立（终端 2）

```bash
ros2 service call /bpx/stand_up std_srvs/srv/Trigger
```

等待机器人完全站起（约 2-3 秒）。

**第三步**：启动键盘遥控（终端 2）

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**按键说明**：

```
   u    i    o          i=前进
   j    k    l          j=左转  l=右转  k=停止
   m    ,    .          ,=后退

q / z : 增减最大线速度 0.1 m/s
w / x : 增减最大角速度 0.1 rad/s
```

**第四步**：结束控制

1. 按 `k` 停止运动
2. 按 `Ctrl+C` 退出键盘遥控
3. 趴下：

```bash
ros2 service call /bpx/sit_down std_srvs/srv/Trigger
```

### 程序化控制

直接发布 `/cmd_vel` 话题即可控制：

```bash
# 前进 0.3 m/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}" --rate 10

# 原地左转 0.5 rad/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}" --rate 10

# 前进 + 左转
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.5}}" --rate 10

# 停止
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}" --rate 10
```

> 超过 0.5 秒未收到新指令，机器人会自动停止（超时保护）。

---

## 话题

### 订阅

| 话题 | 类型 | 说明 |
|------|------|------|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 速度控制指令 |

`Twist` 字段对应关系：

| 字段 | 含义 | 单位 |
|------|------|------|
| `linear.x` | 前进（+）/ 后退（-） | m/s |
| `linear.y` | 左移（+）/ 右移（-） | m/s |
| `angular.z` | 左转（+）/ 右转（-） | rad/s |

### 发布

| 话题 | 类型 | 说明 |
|------|------|------|
| `/joint_states` | `sensor_msgs/msg/JointState` | 12 个关节位置/速度/力矩 |
| `/imu/data` | `sensor_msgs/msg/Imu` | IMU 四元数、角速度、线加速度 |
| `/odom` | `nav_msgs/msg/Odometry` | 速度积分里程计（机体速度 × IMU 姿态积分） |
| `/bpx/battery` | `std_msgs/msg/UInt8` | 电池百分比（0-100） |
| `/bpx/battery_current` | `std_msgs/msg/Float32` | 电池电流（A） |
| `/bpx/motion_state` | `std_msgs/msg/UInt8` | 运动状态码（6=站立） |
| `/bpx/gait` | `std_msgs/msg/UInt8` | 步态码 |

### TF 广播

| 父坐标系 | 子坐标系 | 数据来源 |
|----------|----------|----------|
| `odom` | `base_link` | 速度积分里程计 |
| `base_footprint` → `base_link` → `torso` → ... 等 | 全部关节 | `robot_state_publisher`（基于 URDF + `/joint_states`） |

启动驱动后可直接在 RViz 中查看完整机器人模型（RobotModel 显示，话题 `/robot_description`）。

### 关节名称

12 个关节，与 URDF 模型完全匹配：

```
fl_hip_roll_joint    fl_hip_pitch_joint    fl_knee_joint     (左前)
fr_hip_roll_joint    fr_hip_pitch_joint    fr_knee_joint     (右前)
hl_hip_roll_joint    hl_hip_pitch_joint    hl_knee_joint     (左后)
hr_hip_roll_joint    hr_hip_pitch_joint    hr_knee_joint     (右后)
```

关节数据可用于 RViz 中实时显示机器人姿态（配合 `robot_state_publisher`）。

---

## 服务

所有服务类型均为 `std_srvs/srv/Trigger`。

| 服务名 | 说明 |
|--------|------|
| `/bpx/stand_up` | 站起 |
| `/bpx/sit_down` | 趴下（自动关闭速度控制） |
| `/bpx/damping` | 阻尼模式（关节放松，自动关闭速度控制） |
| `/bpx/upright` | 直立待机 |

调用示例：

```bash
ros2 service call /bpx/stand_up std_srvs/srv/Trigger
```

成功返回：`success=True, message="Stand up command sent"`

---

## 参数

启动时可通过命令行覆盖参数：

```bash
ros2 launch bpx_driver bpx_driver.launch.py \
  robot_ip:="10.21.20.1" \
  state_upload_rate_hz:=100 \
  motion_command_rate_hz:=50
```

或修改配置文件 `config/bpx_params.yaml`：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `robot_ip` | `192.168.0.1` | 机器狗 IP 地址 |
| `state_upload_rate_hz` | `100` | 状态上传频率（Hz） |
| `motion_command_rate_hz` | `50` | 运动指令发送频率（Hz） |
| `state_publish_rate_hz` | `50` | ROS2 话题发布频率（Hz） |
| `cmd_vel_timeout_sec` | `0.5` | 速度指令超时时间（秒） |
| `odom_frame` | `odom` | 里程计坐标系 |
| `base_frame` | `base_link` | 机体坐标系 |
| `imu_frame` | `imu_link` | IMU 坐标系 |

### 有线连接配置

如果使用有线连接，修改 `config/bpx_params.yaml`：

```yaml
bpx_driver_node:
  ros__parameters:
    robot_ip: "10.21.20.1"   # 改为有线 IP
```

---

## 常见问题

### 1. 连接失败

```
[FATAL] [bpx_driver_node]: Failed to connect to robot at 192.168.0.1
```

- 检查网络连接：`ping 192.168.0.1`
- 确认 IP 地址是否正确（有线 vs 无线）
- 确认机器狗已开机

### 2. shared library 找不到

```
error while loading shared libraries: libbpx_sdk_x86_64.so: cannot open shared object file
```

- 确认已重新构建：`colcon build --packages-select bpx_driver`
- 确认 `.so` 已安装：`ls install/bpx_driver/lib/bpx_driver/*.so`

### 3. 键盘遥控无反应

- 确认已先调用 `/bpx/stand_up` 站起
- 确认 `teleop_twist_keyboard` 的话题与驱动一致：`ros2 topic echo /cmd_vel`
- 检查终端焦点是否在键盘遥控窗口

### 4. 机器狗不动

- 检查 `/bpx/motion_state`：`ros2 topic echo /bpx/motion_state`，值为 6 表示已站立
- 检查是否超时：发送频率不能低于 2 Hz
- 检查速度值是否过小
