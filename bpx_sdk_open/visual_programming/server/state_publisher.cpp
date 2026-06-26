#include "state_publisher.h"

#include <chrono>
#include <iostream>

namespace bpx_vp {

StatePublisher::StatePublisher(RobotBridge& bridge, uint16_t rate_hz)
    : bridge_(bridge), rate_hz_(rate_hz) {
}

StatePublisher::~StatePublisher() {
    stop();
}

void StatePublisher::start(BroadcastCallback broadcast_cb) {
    if (running_) return;

    running_ = true;
    publish_thread_ = std::thread(&StatePublisher::publishThread, this, broadcast_cb);
}

void StatePublisher::stop() {
    running_ = false;
    if (publish_thread_.joinable()) {
        publish_thread_.join();
    }
}

void StatePublisher::setRate(uint16_t rate_hz) {
    rate_hz_ = rate_hz;
}

bool StatePublisher::isRunning() const {
    return running_.load();
}

void StatePublisher::publishThread(BroadcastCallback broadcast_cb) {
    auto interval = std::chrono::milliseconds(1000 / rate_hz_);

    while (running_) {
        try {
            json state = bridge_.getState();
            state["type"] = "state";
            if (broadcast_cb) {
                broadcast_cb(state);
            }
        } catch (const std::exception& e) {
            std::cerr << "[StatePublisher] Error: " << e.what() << std::endl;
        }

        std::this_thread::sleep_for(interval);
    }
}

}  // namespace bpx_vp
