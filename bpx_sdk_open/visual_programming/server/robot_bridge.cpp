#include "robot_bridge.h"
#include "joint_level_control.h"
#include "motion_level_control.h"
#include "request_robot_state.h"

#include <chrono>
#include <iostream>
#include <thread>

namespace bpx_vp {

RobotBridge::RobotBridge()
    : motion_ctrl_(std::make_unique<bpx_sdk::MotionLevelControl>()),
      joint_ctrl_(std::make_unique<bpx_sdk::JointLevelControl>()) {
}

RobotBridge::~RobotBridge() {
    disconnect();
}

void RobotBridge::setNetworkConfig(const NetworkConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);
    net_config_ = config;
}

std::vector<NetworkInterface> RobotBridge::getAvailableInterfaces() const {
    return enumerateInterfaces(net_config_.robot_ip);
}

bool RobotBridge::setInterface(const std::string& interface_name) {
    std::lock_guard<std::mutex> lock(mutex_);
    active_interface_ = interface_name;
    return true;
}

bool RobotBridge::connect() {
    return connect(net_config_.robot_ip, net_config_.local_interface);
}

bool RobotBridge::connect(const std::string& ip, const std::string& interface) {
    // 如果已连接，先断开（避免 UDP 端口冲突）
    if (motion_connected_ || joint_connected_) {
        std::cout << "[RobotBridge] Already connected, disconnecting first..." << std::endl;
        disconnect();
    }

    // 仅快速更新配置（持锁时间极短）
    {
        std::lock_guard<std::mutex> lock(mutex_);
        connecting_ = true;
        net_config_.robot_ip = ip;
        active_interface_ = interface;
    }

    std::cout << "[RobotBridge] Connecting to " << ip
              << " via interface " << interface << std::endl;

    // 配置并连接运动控制（慢操作，不持锁）
    motion_ctrl_->setRobotIp(ip.c_str());
    motion_ctrl_->setRobotStateUploadPort(net_config_.state_udp_port);
    motion_ctrl_->setTcpLocalPort(net_config_.tcp_local_port);
    motion_ctrl_->setRobotStateUploadRate(net_config_.state_upload_rate);
    motion_ctrl_->setMotionCommandRate(net_config_.motion_command_rate);

    if (!motion_ctrl_->connect()) {
        std::cerr << "[RobotBridge] Failed to connect motion control" << std::endl;
        motion_connected_ = false;
        connecting_ = false;
        return false;
    }
    motion_connected_ = true;

    // 配置并连接关节控制（使用独立的 UDP 端口，避免与运动控制冲突）
    joint_ctrl_->setRobotIp(ip.c_str());
    joint_ctrl_->setRobotStateUploadPort(net_config_.joint_state_udp_port);
    joint_ctrl_->setTcpLocalPort(net_config_.tcp_local_port);
    joint_ctrl_->setRobotStateUploadRate(net_config_.state_upload_rate);

    if (!joint_ctrl_->connect()) {
        std::cerr << "[RobotBridge] Warning: joint control connect failed (non-fatal)" << std::endl;
        joint_connected_ = false;
    } else {
        joint_connected_ = true;
    }

    connecting_ = false;
    std::cout << "[RobotBridge] Connected successfully" << std::endl;
    return true;
}

void RobotBridge::connectAsync(const std::string& ip, const std::string& interface,
                                std::function<void(bool success)> callback) {
    // 在后台线程中执行连接（避免阻塞 HTTP 服务器）
    std::thread([this, ip, interface, callback]() {
        bool result = this->connect(ip, interface);
        if (callback) callback(result);
    }).detach();
}

void RobotBridge::disconnect() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (motion_connected_) {
        motion_ctrl_->disconnect();
        motion_connected_ = false;
    }
    if (joint_connected_) {
        joint_ctrl_->disconnect();
        joint_connected_ = false;
    }
    connecting_ = false;
    std::cout << "[RobotBridge] Disconnected" << std::endl;
}

bool RobotBridge::isConnecting() const {
    return connecting_.load();
}

bool RobotBridge::isConnected() const {
    return motion_connected_.load();
}

std::string RobotBridge::getActiveInterface() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return active_interface_;
}

std::string RobotBridge::getRobotIp() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return net_config_.robot_ip;
}

// --- 运动控制 ---

void RobotBridge::ensureMotionConnected() {
    if (!motion_connected_) {
        throw std::runtime_error("Robot not connected (motion control)");
    }
}

void RobotBridge::ensureJointConnected() {
    if (!joint_connected_) {
        throw std::runtime_error("Robot not connected (joint control)");
    }
}

bool RobotBridge::standUp() {
    std::lock_guard<std::mutex> lock(mutex_);
    ensureMotionConnected();
    // 状态转换时关闭速度控制标志
    if (velocity_control_enabled_) {
        motion_ctrl_->setVelocityControlFlag(false);
        velocity_control_enabled_ = false;
    }
    return motion_ctrl_->setStandUp();
}

bool RobotBridge::sitDown() {
    std::lock_guard<std::mutex> lock(mutex_);
    ensureMotionConnected();
    // 状态转换时关闭速度控制标志
    if (velocity_control_enabled_) {
        motion_ctrl_->setVelocityControlFlag(false);
        velocity_control_enabled_ = false;
    }
    return motion_ctrl_->setSitDown();
}

bool RobotBridge::setDamping() {
    std::lock_guard<std::mutex> lock(mutex_);
    ensureMotionConnected();
    // 状态转换时关闭速度控制标志
    if (velocity_control_enabled_) {
        motion_ctrl_->setVelocityControlFlag(false);
        velocity_control_enabled_ = false;
    }
    return motion_ctrl_->setDamping();
}

bool RobotBridge::setUpright() {
    // SDK 没有 setUpright，使用 damping 作为替代
    std::lock_guard<std::mutex> lock(mutex_);
    ensureMotionConnected();
    return motion_ctrl_->setDamping();
}

bool RobotBridge::setVelocity(float x, float y, float yaw) {
    std::lock_guard<std::mutex> lock(mutex_);
    ensureMotionConnected();
    // SDK 要求先启用速度控制标志，否则速度指令无效
    if (!velocity_control_enabled_) {
        motion_ctrl_->setVelocityControlFlag(true);
        velocity_control_enabled_ = true;
        std::cout << "[RobotBridge] Auto-enabled velocity control flag" << std::endl;
    }
    current_vx_ = x;
    current_vy_ = y;
    current_vyaw_ = yaw;
    return motion_ctrl_->setVelocity(x, y, yaw);
}

bool RobotBridge::setVelocityControlFlag(bool enabled) {
    std::lock_guard<std::mutex> lock(mutex_);
    ensureMotionConnected();
    motion_ctrl_->setVelocityControlFlag(enabled);
    velocity_control_enabled_ = enabled;
    if (!enabled) {
        current_vx_ = current_vy_ = current_vyaw_ = 0;
    }
    std::cout << "[RobotBridge] Velocity control flag: " << (enabled ? "ON" : "OFF") << std::endl;
    return true;
}

bool RobotBridge::setZeroPositionsFlag() {
    std::lock_guard<std::mutex> lock(mutex_);
    ensureMotionConnected();
    motion_ctrl_->setZeroPositionsFlag();
    return true;
}

// --- 步态控制 ---

bool RobotBridge::setWalk() {
    std::lock_guard<std::mutex> lock(mutex_);
    ensureMotionConnected();
    motion_ctrl_->setWalk();
    return true;
}

bool RobotBridge::setRunning() {
    std::lock_guard<std::mutex> lock(mutex_);
    ensureMotionConnected();
    motion_ctrl_->setRunning();
    return true;
}

bool RobotBridge::setLeftFlip() {
    std::lock_guard<std::mutex> lock(mutex_);
    ensureMotionConnected();
    motion_ctrl_->setLeftFlip();
    return true;
}

bool RobotBridge::setRightFlip() {
    std::lock_guard<std::mutex> lock(mutex_);
    ensureMotionConnected();
    motion_ctrl_->setRightFlip();
    return true;
}

bool RobotBridge::setBipedal() {
    std::lock_guard<std::mutex> lock(mutex_);
    ensureMotionConnected();
    motion_ctrl_->setBipedal();
    return true;
}

bool RobotBridge::setInvBipedal() {
    std::lock_guard<std::mutex> lock(mutex_);
    ensureMotionConnected();
    motion_ctrl_->setInvBipedal();
    return true;
}

bool RobotBridge::setPronk() {
    std::lock_guard<std::mutex> lock(mutex_);
    ensureMotionConnected();
    motion_ctrl_->setPronk();
    return true;
}

bool RobotBridge::setPace() {
    std::lock_guard<std::mutex> lock(mutex_);
    ensureMotionConnected();
    motion_ctrl_->setPace();
    return true;
}

bool RobotBridge::setBound() {
    std::lock_guard<std::mutex> lock(mutex_);
    ensureMotionConnected();
    motion_ctrl_->setBound();
    return true;
}

bool RobotBridge::setForwardVelocity(float x) {
    std::lock_guard<std::mutex> lock(mutex_);
    ensureMotionConnected();
    if (!velocity_control_enabled_) {
        motion_ctrl_->setVelocityControlFlag(true);
        velocity_control_enabled_ = true;
    }
    current_vx_ = x;
    return motion_ctrl_->setVelocity(current_vx_, current_vy_, current_vyaw_);
}

bool RobotBridge::setLateralVelocity(float y) {
    std::lock_guard<std::mutex> lock(mutex_);
    ensureMotionConnected();
    if (!velocity_control_enabled_) {
        motion_ctrl_->setVelocityControlFlag(true);
        velocity_control_enabled_ = true;
    }
    current_vy_ = y;
    return motion_ctrl_->setVelocity(current_vx_, current_vy_, current_vyaw_);
}

bool RobotBridge::setTurnVelocity(float yaw) {
    std::lock_guard<std::mutex> lock(mutex_);
    ensureMotionConnected();
    if (!velocity_control_enabled_) {
        motion_ctrl_->setVelocityControlFlag(true);
        velocity_control_enabled_ = true;
    }
    current_vyaw_ = yaw;
    return motion_ctrl_->setVelocity(current_vx_, current_vy_, current_vyaw_);
}

// --- 关节控制 ---

bool RobotBridge::setJointPosition(const std::array<float, 12>& pos) {
    std::lock_guard<std::mutex> lock(mutex_);
    ensureJointConnected();
    return joint_ctrl_->setJointPosition(pos);
}

bool RobotBridge::setJointGains(float kp, float kd) {
    std::lock_guard<std::mutex> lock(mutex_);
    ensureJointConnected();
    std::array<float, 12> kp_arr, kd_arr;
    kp_arr.fill(kp);
    kd_arr.fill(kd);
    return joint_ctrl_->setJointKp(kp_arr) && joint_ctrl_->setJointKd(kd_arr);
}

bool RobotBridge::setJointCommand(const std::array<float, 12>& kp,
                                   const std::array<float, 12>& pos,
                                   const std::array<float, 12>& kd,
                                   const std::array<float, 12>& vel,
                                   const std::array<float, 12>& tff) {
    std::lock_guard<std::mutex> lock(mutex_);
    ensureJointConnected();
    return joint_ctrl_->setJointCommand(kp, pos, kd, vel, tff);
}

bool RobotBridge::setZeroJointCommand() {
    std::lock_guard<std::mutex> lock(mutex_);
    ensureJointConnected();
    return joint_ctrl_->setZeroJointCommand();
}

bool RobotBridge::smoothMoveTo(const std::array<float, 12>& target, float duration_sec) {
    // 读取当前位置（不锁，因为 getJointPosition 是线程安全的）
    float current[12] = {};
    if (!joint_ctrl_->getJointPositionHighRate(current)) {
        std::cerr << "[RobotBridge] Cannot read current joint position for smooth move" << std::endl;
        return false;
    }

    auto start = std::chrono::steady_clock::now();
    auto duration = std::chrono::microseconds(static_cast<int64_t>(duration_sec * 1e6f));
    std::array<float, 12> cmd_pos;

    while (true) {
        auto elapsed = std::chrono::steady_clock::now() - start;
        float alpha = static_cast<float>(
            std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count()) /
            static_cast<float>(
            std::chrono::duration_cast<std::chrono::microseconds>(duration).count());

        if (alpha > 1.0f) alpha = 1.0f;

        for (int i = 0; i < 12; ++i) {
            cmd_pos[i] = current[i] + alpha * (target[i] - current[i]);
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!joint_ctrl_->setJointPosition(cmd_pos)) {
                std::cerr << "[RobotBridge] Failed to send smooth move command" << std::endl;
                return false;
            }
        }

        if (alpha >= 1.0f) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return true;
}

// --- 状态读取 ---

json RobotBridge::getState() const {
    json state;
    state["connected"] = motion_connected_.load();
    state["motionConnected"] = motion_connected_.load();
    state["jointConnected"] = joint_connected_.load();
    state["connecting"] = connecting_.load();

    {
        std::lock_guard<std::mutex> lock(mutex_);
        state["robotIp"] = net_config_.robot_ip;
        state["interface"] = active_interface_;
    }

    // 正在连接中或未连接，只返回基本信息
    if (connecting_.load() || !motion_connected_.load()) return state;

    // 运动状态
    uint8_t motion_state = 0, gait = 0, sub_gait = 0, battery = 0;
    uint8_t last_motion_state = 0, last_gait = 0;
    float battery_current = 0.0f;
    float body_vel[3] = {};
    float imu_rpy[3] = {};
    float imu_quat[4] = {};
    float imu_acc[3] = {};
    float imu_omega[3] = {};
    float leg_odom[3] = {};
    float joint_pos[12] = {};
    float joint_vel[12] = {};
    float joint_tau[12] = {};
    float motor_temp[12] = {};
    float driver_temp[12] = {};
    float max_vel[3] = {};

    if (motion_ctrl_->getCurrentMotionState(&motion_state))
        state["motionState"] = motion_state;
    if (motion_ctrl_->getCurrentGait(&gait))
        state["gait"] = gait;
    if (motion_ctrl_->getSubGait(&sub_gait))
        state["subGait"] = sub_gait;
    if (motion_ctrl_->getLastMotionState(&last_motion_state))
        state["lastMotionState"] = last_motion_state;
    if (motion_ctrl_->getLastGait(&last_gait))
        state["lastGait"] = last_gait;
    if (motion_ctrl_->getBatteryLevel(&battery))
        state["battery"] = battery;
    if (motion_ctrl_->getBatteryCurrent(&battery_current))
        state["batteryCurrent"] = battery_current;
    if (motion_ctrl_->getCurrentVelocityBody(body_vel))
        state["bodyVelocity"] = {body_vel[0], body_vel[1], body_vel[2]};
    if (motion_ctrl_->getImuRpy(imu_rpy))
        state["imuRpy"] = {imu_rpy[0], imu_rpy[1], imu_rpy[2]};
    if (motion_ctrl_->getImuQuat(imu_quat))
        state["imuQuat"] = {imu_quat[0], imu_quat[1], imu_quat[2], imu_quat[3]};
    if (motion_ctrl_->getImuAcc(imu_acc))
        state["imuAcc"] = {imu_acc[0], imu_acc[1], imu_acc[2]};
    if (motion_ctrl_->getImuOmega(imu_omega))
        state["imuOmega"] = {imu_omega[0], imu_omega[1], imu_omega[2]};
    if (motion_ctrl_->getLegOdom(leg_odom))
        state["legOdom"] = {leg_odom[0], leg_odom[1], leg_odom[2]};
    if (motion_ctrl_->getMaxVelocity(max_vel))
        state["maxVelocity"] = {max_vel[0], max_vel[1], max_vel[2]};

    // 关节数据
    if (joint_connected_) {
        if (joint_ctrl_->getJointPositionHighRate(joint_pos)) {
            state["jointPos"] = std::vector<float>(joint_pos, joint_pos + 12);
        }
        if (joint_ctrl_->getJointVelocityHighRate(joint_vel)) {
            state["jointVel"] = std::vector<float>(joint_vel, joint_vel + 12);
        }
        if (joint_ctrl_->getJointTorqueHighRate(joint_tau)) {
            state["jointTorque"] = std::vector<float>(joint_tau, joint_tau + 12);
        }
    } else {
        // 关节控制未连接，尝试从运动控制获取关节数据
        if (motion_ctrl_->getJointPosition(joint_pos)) {
            state["jointPos"] = std::vector<float>(joint_pos, joint_pos + 12);
        }
        if (motion_ctrl_->getJointVelocity(joint_vel)) {
            state["jointVel"] = std::vector<float>(joint_vel, joint_vel + 12);
        }
        if (motion_ctrl_->getJointTorque(joint_tau)) {
            state["jointTorque"] = std::vector<float>(joint_tau, joint_tau + 12);
        }
    }

    if (motion_ctrl_->getMotorTemperature(motor_temp))
        state["motorTemp"] = std::vector<float>(motor_temp, motor_temp + 12);
    if (motion_ctrl_->getDriverTemperature(driver_temp))
        state["driverTemp"] = std::vector<float>(driver_temp, driver_temp + 12);

    return state;
}

json RobotBridge::getFullState() const {
    return getState();
}

bool RobotBridge::getJointPosition(float out[12]) const {
    if (!motion_connected_ && !joint_connected_) return false;
    if (joint_connected_) {
        return joint_ctrl_->getJointPositionHighRate(out);
    }
    return motion_ctrl_->getJointPosition(out);
}

}  // namespace bpx_vp
