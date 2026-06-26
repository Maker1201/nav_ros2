#include "command_executor.h"

#include <chrono>
#include <iostream>
#include <thread>

namespace bpx_vp {

CommandExecutor::CommandExecutor(RobotBridge& bridge) : bridge_(bridge) {
}

CommandExecutor::~CommandExecutor() {
    stop();
    if (exec_thread_.joinable()) {
        exec_thread_.join();
    }
}

void CommandExecutor::executeAsync(const json& program) {
    executeAsync(program, nullptr, nullptr);
}

void CommandExecutor::executeAsync(const json& program, ExecStatusCallback status_cb, LogCallback log_cb) {
    if (running_) {
        if (log_cb) log_cb("已有程序在运行中，请先停止");
        return;
    }

    stop_requested_ = false;

    if (exec_thread_.joinable()) {
        exec_thread_.join();
    }

    exec_thread_ = std::thread(&CommandExecutor::executeThread, this, program, status_cb, log_cb);
}

void CommandExecutor::stop() {
    stop_requested_ = true;
    // 紧急停止：发送阻尼模式
    try {
        if (bridge_.isConnected()) {
            bridge_.setDamping();
        }
    } catch (...) {}
}

bool CommandExecutor::isRunning() const {
    return running_.load();
}

void CommandExecutor::executeThread(json program, ExecStatusCallback status_cb, LogCallback log_cb) {
    running_ = true;

    // 使用 RAII 确保 running_ 在任何情况下都会被重置
    struct RunningGuard {
        std::atomic<bool>& flag;
        ExecStatusCallback status_cb;
        std::atomic<bool>& stop_flag;
        ~RunningGuard() {
            if (status_cb) {
                json s;
                s["running"] = false;
                s["finished"] = !stop_flag;
                s["stopped"] = stop_flag.load();
                status_cb(s);
            }
            flag = false;
        }
    } guard{running_, status_cb, stop_requested_};

    if (!program.is_array()) {
        if (log_cb) log_cb("错误：程序必须是指令数组");
        return;
    }

    size_t total = program.size();

    if (status_cb) {
        json s;
        s["running"] = true;
        s["currentStep"] = 0;
        s["totalSteps"] = total;
        status_cb(s);
    }

    for (size_t i = 0; i < total; ++i) {
        if (stop_requested_) {
            if (log_cb) log_cb("程序已被用户停止");
            break;
        }

        const auto& cmd = program[i];
        std::string action = cmd.value("action", "unknown");

        if (log_cb) {
            log_cb("执行步骤 " + std::to_string(i + 1) + "/" + std::to_string(total) + ": " + action);
        }

        if (status_cb) {
            json s;
            s["running"] = true;
            s["currentStep"] = i + 1;
            s["totalSteps"] = total;
            s["currentAction"] = action;
            status_cb(s);
        }

        try {
            json result = executeCommand(cmd);
            if (result.contains("error")) {
                if (log_cb) log_cb("⚠ 步骤 " + std::to_string(i + 1) + " 错误: " + result["error"].get<std::string>());
                // 记录错误但继续执行后续步骤
            }
        } catch (const std::exception& e) {
            if (log_cb) log_cb("⚠ 步骤 " + std::to_string(i + 1) + " 异常: " + std::string(e.what()));
            // 记录异常但继续执行后续步骤
        }
    }
}

json CommandExecutor::executeCommand(const json& cmd) {
    return executeSingle(cmd);
}

json CommandExecutor::executeSingle(const json& cmd) {
    json result;
    std::string action = cmd.value("action", "unknown");

    try {
        if (action == "connect") {
            std::string ip = cmd.value("ip", "10.21.20.1");
            std::string iface = cmd.value("interface", "auto");
            // 如果已连接到同一 IP，跳过重复连接
            if (bridge_.isConnected() && bridge_.getRobotIp() == ip) {
                result["success"] = true;
                result["skipped"] = true;
            } else if (bridge_.connect(ip, iface)) {
                result["success"] = true;
            } else {
                result["error"] = "连接失败";
            }
        } else if (action == "disconnect") {
            bridge_.disconnect();
            result["success"] = true;
        } else if (action == "standUp") {
            result["success"] = bridge_.standUp();
        } else if (action == "sitDown") {
            result["success"] = bridge_.sitDown();
        } else if (action == "damping") {
            result["success"] = bridge_.setDamping();
        } else if (action == "upright") {
            result["success"] = bridge_.setUpright();
        } else if (action == "setVelocity") {
            float x = cmd.value("x", 0.0f);
            float y = cmd.value("y", 0.0f);
            float yaw = cmd.value("yaw", 0.0f);
            result["success"] = bridge_.setVelocity(x, y, yaw);
        } else if (action == "setForwardVelocity") {
            float x = cmd.value("value", 0.0f);
            result["success"] = bridge_.setForwardVelocity(x);
        } else if (action == "setLateralVelocity") {
            float y = cmd.value("value", 0.0f);
            result["success"] = bridge_.setLateralVelocity(y);
        } else if (action == "setTurnVelocity") {
            float yaw = cmd.value("value", 0.0f);
            result["success"] = bridge_.setTurnVelocity(yaw);
        } else if (action == "setVelocityControlFlag") {
            bool enabled = cmd.value("enabled", false);
            result["success"] = bridge_.setVelocityControlFlag(enabled);
        } else if (action == "zeroPositions") {
            result["success"] = bridge_.setZeroPositionsFlag();
        } else if (action == "walk") {
            result["success"] = bridge_.setWalk();
        } else if (action == "running") {
            result["success"] = bridge_.setRunning();
        } else if (action == "leftFlip") {
            result["success"] = bridge_.setLeftFlip();
        } else if (action == "rightFlip") {
            result["success"] = bridge_.setRightFlip();
        } else if (action == "bipedal") {
            result["success"] = bridge_.setBipedal();
        } else if (action == "invBipedal") {
            result["success"] = bridge_.setInvBipedal();
        } else if (action == "pronk") {
            result["success"] = bridge_.setPronk();
        } else if (action == "pace") {
            result["success"] = bridge_.setPace();
        } else if (action == "bound") {
            result["success"] = bridge_.setBound();
        } else if (action == "wait") {
            float seconds = cmd.value("seconds", 1.0f);
            int ms = static_cast<int>(seconds * 1000);
            auto start = std::chrono::steady_clock::now();
            while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(ms)) {
                if (stop_requested_) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            result["success"] = true;
        } else if (action == "setJointPosition") {
            std::array<float, 12> pos{};
            if (cmd.contains("position") && cmd["position"].is_array()) {
                auto arr = cmd["position"];
                for (size_t i = 0; i < 12 && i < arr.size(); ++i) {
                    pos[i] = arr[i].get<float>();
                }
            }
            result["success"] = bridge_.setJointPosition(pos);
        } else if (action == "setJointGains") {
            float kp = cmd.value("kp", 100.0f);
            float kd = cmd.value("kd", 2.0f);
            result["success"] = bridge_.setJointGains(kp, kd);
        } else if (action == "smoothMove") {
            std::array<float, 12> target{};
            if (cmd.contains("target") && cmd["target"].is_array()) {
                auto arr = cmd["target"];
                for (size_t i = 0; i < 12 && i < arr.size(); ++i) {
                    target[i] = arr[i].get<float>();
                }
            }
            float duration = cmd.value("duration", 2.0f);
            result["success"] = bridge_.smoothMoveTo(target, duration);
        } else if (action == "smoothMoveSingleLeg") {
            // 平滑移动单条腿，保留其他腿的当前位置
            int leg = cmd.value("leg", 0);
            float abad = cmd.value("abad", 0.0f);
            float hip = cmd.value("hip", 0.0f);
            float knee = cmd.value("knee", 0.0f);
            float duration = cmd.value("duration", 2.0f);
            if (leg < 0 || leg > 3) {
                result["error"] = "腿编号无效，应为 0-3";
            } else {
                std::array<float, 12> target{};
                float current[12] = {};
                if (bridge_.getJointPosition(current)) {
                    for (int j = 0; j < 12; ++j) target[j] = current[j];
                }
                target[3 * leg + 0] = abad;
                target[3 * leg + 1] = hip;
                target[3 * leg + 2] = knee;
                result["success"] = bridge_.smoothMoveTo(target, duration);
            }
        } else if (action == "zeroJoints") {
            result["success"] = bridge_.setZeroJointCommand();
        } else if (action == "setLegJoint") {
            // 设置单条腿的关节角度（保留其他腿的当前位置）
            int leg = cmd.value("leg", 0);  // 0=左前, 1=右前, 2=左后, 3=右后
            float abad = cmd.value("abad", 0.0f);
            float hip = cmd.value("hip", 0.0f);
            float knee = cmd.value("knee", 0.0f);
            if (leg < 0 || leg > 3) {
                result["error"] = "腿编号无效，应为 0-3";
            } else {
                // 读取当前关节位置，只修改目标腿的 3 个关节
                std::array<float, 12> pos{};
                float current[12] = {};
                if (bridge_.getJointPosition(current)) {
                    for (int j = 0; j < 12; ++j) pos[j] = current[j];
                }
                pos[3 * leg + 0] = abad;
                pos[3 * leg + 1] = hip;
                pos[3 * leg + 2] = knee;
                result["success"] = bridge_.setJointPosition(pos);
            }
        } else if (action == "emergencyStop") {
            bridge_.setDamping();
            stop_requested_ = true;
            result["success"] = true;
        } else if (action == "loop") {
            // 循环指令由前端展开，后端收到时直接执行 body 内容（fallback）
            if (cmd.contains("body") && cmd["body"].is_array()) {
                int times = 1;
                if (cmd.contains("times")) {
                    if (cmd["times"].is_number()) {
                        times = cmd["times"].get<int>();
                    } else if (cmd["times"].is_string()) {
                        times = std::stoi(cmd["times"].get<std::string>());
                    }
                }
                for (int t = 0; t < times && !stop_requested_; ++t) {
                    for (const auto& bodyCmd : cmd["body"]) {
                        if (stop_requested_) break;
                        json r = executeSingle(bodyCmd);
                        if (r.contains("error")) {
                            result["error"] = r["error"];
                            break;
                        }
                    }
                }
                if (!result.contains("error")) result["success"] = true;
            }
        } else if (action == "if") {
            // 条件指令由前端评估，后端收到时直接执行 then 分支（fallback）
            if (cmd.contains("then") && cmd["then"].is_array()) {
                for (const auto& bodyCmd : cmd["then"]) {
                    if (stop_requested_) break;
                    json r = executeSingle(bodyCmd);
                    if (r.contains("error")) {
                        result["error"] = r["error"];
                        break;
                    }
                }
                if (!result.contains("error")) result["success"] = true;
            }
        } else if (action == "repeatUntil") {
            // 由前端展开，后端 fallback：最多执行 100 次
            if (cmd.contains("body") && cmd["body"].is_array()) {
                for (int iter = 0; iter < 100 && !stop_requested_; ++iter) {
                    for (const auto& bodyCmd : cmd["body"]) {
                        if (stop_requested_) break;
                        json r = executeSingle(bodyCmd);
                        if (r.contains("error")) {
                            result["error"] = r["error"];
                            break;
                        }
                    }
                    if (result.contains("error")) break;
                }
                if (!result.contains("error")) result["success"] = true;
            }
        } else {
            result["error"] = "未知指令: " + action;
        }
    } catch (const std::exception& e) {
        result["error"] = std::string("执行异常: ") + e.what();
    }

    return result;
}

}  // namespace bpx_vp
