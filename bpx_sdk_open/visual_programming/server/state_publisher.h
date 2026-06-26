#ifndef BPX_VISUAL_PROGRAMMING_STATE_PUBLISHER_H_
#define BPX_VISUAL_PROGRAMMING_STATE_PUBLISHER_H_

#include "json.hpp"
#include "robot_bridge.h"

#include <atomic>
#include <functional>
#include <thread>

namespace bpx_vp {

using json = nlohmann::json;
using BroadcastCallback = std::function<void(const json&)>;

class StatePublisher {
public:
    StatePublisher(RobotBridge& bridge, uint16_t rate_hz = 10);
    ~StatePublisher();

    // 禁止拷贝
    StatePublisher(const StatePublisher&) = delete;
    StatePublisher& operator=(const StatePublisher&) = delete;

    // 启动/停止状态推送
    void start(BroadcastCallback broadcast_cb);
    void stop();

    // 设置推送频率
    void setRate(uint16_t rate_hz);

    bool isRunning() const;

private:
    void publishThread(BroadcastCallback broadcast_cb);

    RobotBridge& bridge_;
    uint16_t rate_hz_;
    std::atomic<bool> running_{false};
    std::thread publish_thread_;
};

}  // namespace bpx_vp

#endif  // BPX_VISUAL_PROGRAMMING_STATE_PUBLISHER_H_
