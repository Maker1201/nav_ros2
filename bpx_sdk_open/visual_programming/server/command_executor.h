#ifndef BPX_VISUAL_PROGRAMMING_COMMAND_EXECUTOR_H_
#define BPX_VISUAL_PROGRAMMING_COMMAND_EXECUTOR_H_

#include "json.hpp"
#include "robot_bridge.h"

#include <atomic>
#include <functional>
#include <string>
#include <thread>

namespace bpx_vp {

using json = nlohmann::json;

// 执行状态回调
using ExecStatusCallback = std::function<void(const json& status)>;
using LogCallback = std::function<void(const std::string& message)>;

class CommandExecutor {
public:
    CommandExecutor(RobotBridge& bridge);
    ~CommandExecutor();

    // 禁止拷贝
    CommandExecutor(const CommandExecutor&) = delete;
    CommandExecutor& operator=(const CommandExecutor&) = delete;

    // 执行 Blockly 生成的 JSON 程序（异步）
    void executeAsync(const json& program);
    void executeAsync(const json& program, ExecStatusCallback status_cb, LogCallback log_cb);

    // 紧急停止
    void stop();

    // 是否正在运行
    bool isRunning() const;

    // 执行单条指令（同步）
    json executeSingle(const json& command);

private:
    void executeThread(json program, ExecStatusCallback status_cb, LogCallback log_cb);
    json executeCommand(const json& cmd);

    RobotBridge& bridge_;
    std::atomic<bool> running_{false};
    std::atomic<bool> stop_requested_{false};
    std::thread exec_thread_;
};

}  // namespace bpx_vp

#endif  // BPX_VISUAL_PROGRAMMING_COMMAND_EXECUTOR_H_
