#ifndef BPX_VISUAL_PROGRAMMING_ROBOT_BRIDGE_H_
#define BPX_VISUAL_PROGRAMMING_ROBOT_BRIDGE_H_

#include "network_config.h"
#include "json.hpp"

#include <array>
#include <atomic>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

// Forward declare SDK classes to avoid header dependency
namespace bpx_sdk {
class MotionLevelControl;
class JointLevelControl;
}

namespace bpx_vp {

using json = nlohmann::json;

class RobotBridge {
public:
    RobotBridge();
    ~RobotBridge();

    // 禁止拷贝
    RobotBridge(const RobotBridge&) = delete;
    RobotBridge& operator=(const RobotBridge&) = delete;

    // 网络配置
    void setNetworkConfig(const NetworkConfig& config);
    std::vector<NetworkInterface> getAvailableInterfaces() const;
    bool setInterface(const std::string& interface_name);

    // 连接管理
    bool connect();
    bool connect(const std::string& ip, const std::string& interface = "auto");
    void connectAsync(const std::string& ip, const std::string& interface,
                      std::function<void(bool success)> callback);
    void disconnect();
    bool isConnected() const;
    bool isConnecting() const;
    std::string getActiveInterface() const;
    std::string getRobotIp() const;

    // 运动控制（高级）
    bool standUp();
    bool sitDown();
    bool setDamping();
    bool setUpright();
    bool setZeroPositionsFlag();
    bool setVelocity(float x, float y, float yaw);
    bool setForwardVelocity(float x);
    bool setLateralVelocity(float y);
    bool setTurnVelocity(float yaw);
    bool setVelocityControlFlag(bool enabled);

    // 步态控制
    bool setWalk();
    bool setRunning();
    bool setLeftFlip();
    bool setRightFlip();
    bool setBipedal();
    bool setInvBipedal();
    bool setPronk();
    bool setPace();
    bool setBound();

    // 关节控制（低级）
    bool setJointPosition(const std::array<float, 12>& pos);
    bool setJointGains(float kp, float kd);
    bool setJointCommand(const std::array<float, 12>& kp,
                         const std::array<float, 12>& pos,
                         const std::array<float, 12>& kd,
                         const std::array<float, 12>& vel,
                         const std::array<float, 12>& tff);
    bool setZeroJointCommand();
    bool smoothMoveTo(const std::array<float, 12>& target, float duration_sec);

    // 状态读取
    json getState() const;
    json getFullState() const;
    bool getJointPosition(float out[12]) const;

private:
    void ensureMotionConnected();
    void ensureJointConnected();

    NetworkConfig net_config_;
    std::string active_interface_;

    std::unique_ptr<bpx_sdk::MotionLevelControl> motion_ctrl_;
    std::unique_ptr<bpx_sdk::JointLevelControl> joint_ctrl_;

    mutable std::mutex mutex_;
    std::atomic<bool> motion_connected_{false};
    std::atomic<bool> joint_connected_{false};
    std::atomic<bool> connecting_{false};
    bool velocity_control_enabled_{false};
    float current_vx_{0}, current_vy_{0}, current_vyaw_{0};
};

}  // namespace bpx_vp

#endif  // BPX_VISUAL_PROGRAMMING_ROBOT_BRIDGE_H_
