#include "command_executor.h"
#include "network_config.h"
#include "robot_bridge.h"
#include "state_publisher.h"

#include "httplib.h"
#include "json.hpp"

#include <atomic>
#include <iostream>
#include <mutex>
#include <signal.h>
#include <string>

namespace {

using json = nlohmann::json;

bpx_vp::RobotBridge* g_bridge = nullptr;
bpx_vp::CommandExecutor* g_executor = nullptr;
bpx_vp::StatePublisher* g_publisher = nullptr;

std::atomic<bool> g_running{true};

// 日志缓冲
std::vector<json> g_log_buffer;
std::mutex g_log_mutex;
static constexpr size_t MAX_LOG_BUFFER = 200;

void signalHandler(int) {
    g_running = false;
    if (g_bridge) {
        g_bridge->disconnect();
    }
}

void addLog(const std::string& message, const std::string& level = "info") {
    json entry;
    entry["type"] = "log";
    entry["level"] = level;
    entry["message"] = message;
    entry["time"] = std::time(nullptr);

    std::lock_guard<std::mutex> lock(g_log_mutex);
    g_log_buffer.push_back(entry);
    if (g_log_buffer.size() > MAX_LOG_BUFFER) {
        g_log_buffer.erase(g_log_buffer.begin());
    }
    std::cout << "[" << level << "] " << message << std::endl;
}

json processCommand(const json& msg) {
    json resp;
    std::string type = msg.value("type", "");

    if (type == "execute") {
        if (msg.contains("program")) {
            g_executor->executeAsync(msg["program"],
                [](const json& status) {
                    json entry;
                    entry["type"] = "execStatus";
                    entry["running"] = status.value("running", false);
                    entry["currentStep"] = status.value("currentStep", 0);
                    entry["totalSteps"] = status.value("totalSteps", 0);
                    entry["currentAction"] = status.value("currentAction", "");
                    entry["finished"] = status.value("finished", false);
                    entry["stopped"] = status.value("stopped", false);

                    std::lock_guard<std::mutex> lock(g_log_mutex);
                    g_log_buffer.push_back(entry);
                    if (g_log_buffer.size() > MAX_LOG_BUFFER) {
                        g_log_buffer.erase(g_log_buffer.begin());
                    }
                },
                [](const std::string& log) { addLog(log); });
            resp["success"] = true;
        } else {
            resp["error"] = "缺少 program 字段";
        }
    } else if (type == "stop") {
        g_executor->stop();
        addLog("已发送停止指令");
        resp["success"] = true;
    } else if (type == "getNetworkInterfaces") {
        auto ifaces = g_bridge->getAvailableInterfaces();
        resp["type"] = "networkInterfaces";
        resp["interfaces"] = json::array();
        for (const auto& iface : ifaces) {
            json j;
            j["name"] = iface.name;
            j["ip"] = iface.ip;
            j["mac"] = iface.mac;
            j["type"] = iface.type;
            j["up"] = iface.is_up;
            j["sameSubnet"] = iface.same_subnet_as_robot;
            resp["interfaces"].push_back(j);
        }
        resp["robotIp"] = g_bridge->getRobotIp();
        resp["activeInterface"] = g_bridge->getActiveInterface();
    } else if (type == "setNetworkInterface") {
        std::string iface = msg.value("interface", "auto");
        g_bridge->setInterface(iface);
        resp["type"] = "networkConfigUpdated";
        resp["success"] = true;
        resp["message"] = "已切换到 " + iface;
    } else if (type == "updateNetworkConfig") {
        bpx_vp::NetworkConfig config;
        if (msg.contains("robotIp")) config.robot_ip = msg["robotIp"];
        if (msg.contains("interface")) config.local_interface = msg["interface"];
        if (msg.contains("stateUploadRate")) config.state_upload_rate = msg["stateUploadRate"];
        if (msg.contains("motionCommandRate")) config.motion_command_rate = msg["motionCommandRate"];
        g_bridge->setNetworkConfig(config);
        resp["type"] = "networkConfigUpdated";
        resp["success"] = true;
        resp["message"] = "网络配置已更新";
    } else if (type == "connect") {
        std::string ip = msg.value("ip", "10.21.20.1");
        std::string iface = msg.value("interface", "auto");
        addLog("正在连接到 " + ip + " (接口: " + iface + ")...");

        // 先检测连通性
        bool reachable = bpx_vp::checkRobotConnectivity(ip, 1000);
        if (!reachable) {
            // 列出可用接口帮助诊断
            auto ifaces = g_bridge->getAvailableInterfaces();
            std::string available;
            for (const auto& i : ifaces) {
                if (!available.empty()) available += ", ";
                available += i.name + "(" + i.ip + "/" + i.type + ")";
            }
            addLog("⚠ 目标 " + ip + " 不可达。可用接口: " + available, "warn");
            addLog("请确认: 1) 机器狗已开机 2) 网线已连接或在同一 WiFi", "warn");
        }

        // 异步连接，避免阻塞 HTTP 服务器
        g_bridge->connectAsync(ip, iface, [ip](bool success) {
            if (success) {
                addLog("✅ 已连接到 " + ip, "success");
            } else {
                addLog("❌ 连接失败: " + ip + "。请检查网络连接和机器狗状态。", "error");
            }
        });
        resp["type"] = "connecting";
        resp["success"] = true;
        resp["message"] = "正在连接...";
    } else if (type == "disconnect") {
        g_bridge->disconnect();
        addLog("已断开连接");
        resp["success"] = true;
    } else if (type == "checkConnectivity") {
        std::string ip = msg.value("ip", "10.21.20.1");
        bool ok = bpx_vp::checkRobotConnectivity(ip);
        resp["type"] = "connectivityResult";
        resp["ip"] = ip;
        resp["reachable"] = ok;
    } else if (msg.contains("action")) {
        // 单步指令
        resp = g_executor->executeSingle(msg);
        if (resp.contains("error")) {
            addLog("指令执行错误: " + resp["error"].get<std::string>(), "error");
        }
    } else {
        resp["error"] = "未知消息类型: " + type;
    }

    return resp;
}

}  // namespace

int main(int argc, char** argv) {
    // 解析命令行参数
    bpx_vp::NetworkConfig config = bpx_vp::parseArgs(argc, argv);

    std::cout << "=== BPX Visual Programming Server ===" << std::endl;
    std::cout << "Robot IP:     " << config.robot_ip << std::endl;
    std::cout << "Interface:    " << config.local_interface << std::endl;
    std::cout << "Web Port:     " << config.web_server_port << std::endl;
    std::cout << "State Rate:   " << config.state_upload_rate << " Hz" << std::endl;
    std::cout << "Command Rate: " << config.motion_command_rate << " Hz" << std::endl;

    // 检测端口是否已被占用（防止多实例）
    {
        httplib::Client test_client("127.0.0.1", config.web_server_port);
        test_client.set_connection_timeout(0, 100000);  // 100ms
        auto res = test_client.Get("/api/status");
        if (res && res->status == 200) {
            std::cerr << "\n错误: 端口 " << config.web_server_port << " 已被占用，"
                      << "可能已有另一个实例在运行。" << std::endl;
            std::cerr << "请先停止已有进程，或使用 --web-port 指定其他端口。" << std::endl;
            return 1;
        }
    }

    // 初始化模块
    bpx_vp::RobotBridge bridge;
    bpx_vp::CommandExecutor executor(bridge);
    bpx_vp::StatePublisher publisher(bridge, 10);

    g_bridge = &bridge;
    g_executor = &executor;
    g_publisher = &publisher;

    bridge.setNetworkConfig(config);

    // 信号处理
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // 创建 HTTP 服务器
    httplib::Server svr;

    // 静态文件服务（前端页面）
    std::string frontend_dir = "frontend";
    svr.set_mount_point("/", frontend_dir);

    // === API 端点 ===

    // 服务器状态
    svr.Get("/api/status", [](const httplib::Request&, httplib::Response& res) {
        json status;
        status["server"] = "running";
        status["robotConnected"] = g_bridge->isConnected();
        status["executing"] = g_executor->isRunning();
        res.set_content(status.dump(), "application/json");
    });

    // 获取网卡列表
    svr.Get("/api/interfaces", [](const httplib::Request&, httplib::Response& res) {
        auto ifaces = g_bridge->getAvailableInterfaces();
        json resp = json::array();
        for (const auto& iface : ifaces) {
            json j;
            j["name"] = iface.name;
            j["ip"] = iface.ip;
            j["mac"] = iface.mac;
            j["type"] = iface.type;
            j["up"] = iface.is_up;
            j["sameSubnet"] = iface.same_subnet_as_robot;
            resp.push_back(j);
        }
        res.set_content(resp.dump(), "application/json");
    });

    // 机器人连接状态（用于轮询异步连接结果）
    svr.Get("/api/robot-status", [](const httplib::Request&, httplib::Response& res) {
        json status;
        status["connected"] = g_bridge->isConnected();
        status["connecting"] = g_bridge->isConnecting();
        status["robotIp"] = g_bridge->getRobotIp();
        status["interface"] = g_bridge->getActiveInterface();
        res.set_content(status.dump(), "application/json");
    });

    // 连接机器人
    svr.Post("/api/connect", [](const httplib::Request& req, httplib::Response& res) {
        json body = json::parse(req.body);
        std::string ip = body.value("ip", "10.21.20.1");
        std::string iface = body.value("interface", "auto");
        json resp;
        resp["success"] = g_bridge->connect(ip, iface);
        res.set_content(resp.dump(), "application/json");
    });

    // 断开连接
    svr.Post("/api/disconnect", [](const httplib::Request&, httplib::Response& res) {
        g_bridge->disconnect();
        json resp;
        resp["success"] = true;
        res.set_content(resp.dump(), "application/json");
    });

    // 执行程序
    svr.Post("/api/execute", [](const httplib::Request& req, httplib::Response& res) {
        json body = json::parse(req.body);
        if (body.contains("program")) {
            g_executor->executeAsync(body["program"],
                [](const json& status) {
                    // 将执行状态推送到日志缓冲，前端通过 SSE/轮询获取
                    json entry;
                    entry["type"] = "execStatus";
                    entry["running"] = status.value("running", false);
                    entry["currentStep"] = status.value("currentStep", 0);
                    entry["totalSteps"] = status.value("totalSteps", 0);
                    entry["currentAction"] = status.value("currentAction", "");
                    entry["finished"] = status.value("finished", false);
                    entry["stopped"] = status.value("stopped", false);

                    std::lock_guard<std::mutex> lock(g_log_mutex);
                    g_log_buffer.push_back(entry);
                    if (g_log_buffer.size() > MAX_LOG_BUFFER) {
                        g_log_buffer.erase(g_log_buffer.begin());
                    }
                },
                [](const std::string& log) { addLog(log); });
        }
        json resp;
        resp["success"] = true;
        res.set_content(resp.dump(), "application/json");
    });

    // 停止执行
    svr.Post("/api/stop", [](const httplib::Request&, httplib::Response& res) {
        g_executor->stop();
        json resp;
        resp["success"] = true;
        res.set_content(resp.dump(), "application/json");
    });

    // 通用指令处理
    svr.Post("/api/command", [](const httplib::Request& req, httplib::Response& res) {
        json body = json::parse(req.body);
        json result = processCommand(body);
        res.set_content(result.dump(), "application/json");
    });

    // SSE 状态流
    svr.Get("/api/state/stream", [](const httplib::Request&, httplib::Response& res) {
        res.set_header("Content-Type", "text/event-stream");
        res.set_header("Cache-Control", "no-cache");
        res.set_header("Connection", "keep-alive");
        res.set_header("Access-Control-Allow-Origin", "*");

        res.set_chunked_content_provider(
            "text/event-stream",
            [](size_t, httplib::DataSink& sink) {
                while (g_running) {
                    // 推送状态
                    json state = g_bridge->getState();
                    state["type"] = "state";
                    std::string data = "data: " + state.dump() + "\n\n";
                    if (!sink.write(data.c_str(), data.size())) break;

                    // 推送执行状态
                    json exec_status;
                    exec_status["type"] = "execStatus";
                    exec_status["running"] = g_executor->isRunning();
                    data = "data: " + exec_status.dump() + "\n\n";
                    if (!sink.write(data.c_str(), data.size())) break;

                    // 推送新日志
                    {
                        std::lock_guard<std::mutex> lock(g_log_mutex);
                        for (const auto& log : g_log_buffer) {
                            data = "data: " + log.dump() + "\n\n";
                            if (!sink.write(data.c_str(), data.size())) break;
                        }
                        g_log_buffer.clear();
                    }

                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                sink.done();
                return true;
            });
    });

    // 获取日志（只读取，不清空 — SSE 会负责清空）
    svr.Get("/api/logs", [](const httplib::Request&, httplib::Response& res) {
        std::lock_guard<std::mutex> lock(g_log_mutex);
        json logs = g_log_buffer;
        // 不清空缓冲区，避免与 SSE 竞争
        res.set_content(logs.dump(), "application/json");
    });

    std::cout << "\nServer starting on http://0.0.0.0:" << config.web_server_port << std::endl;
    std::cout << "Open browser and navigate to http://localhost:" << config.web_server_port << std::endl;
    std::cout << "Press Ctrl+C to stop.\n" << std::endl;

    // 启动服务器
    if (!svr.listen(config.web_server_bind, config.web_server_port)) {
        std::cerr << "Failed to start server on port " << config.web_server_port << std::endl;
        return 1;
    }

    return 0;
}
