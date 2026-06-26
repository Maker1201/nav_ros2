#include "motion_level_control.h"
#include <chrono>
#include <iostream>
#include <thread>

int main() {
    std::cout << "=== 测试站立指令 ===" << std::endl;

    bpx_sdk::MotionLevelControl ctrl;

    ctrl.setRobotIp("10.21.20.1");
    ctrl.setRobotStateUploadPort(9873);
    ctrl.setTcpLocalPort(0);
    ctrl.setRobotStateUploadRate(100);
    ctrl.setMotionCommandRate(50);

    std::cout << "正在连接..." << std::endl;
    if (!ctrl.connect()) {
        std::cerr << "连接失败!" << std::endl;
        return 1;
    }
    std::cout << "连接成功!" << std::endl;

    // 读取当前状态
    uint8_t state = 0;
    if (ctrl.getCurrentMotionState(&state)) {
        std::cout << "当前运动状态: " << (int)state << std::endl;
    } else {
        std::cout << "无法读取运动状态" << std::endl;
    }

    // 发送站立指令
    std::cout << "发送站立指令..." << std::endl;
    bool ok = ctrl.setStandUp();
    std::cout << "setStandUp() 返回: " << (ok ? "true" : "false") << std::endl;

    // 等待并检查状态
    for (int i = 0; i < 10; i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (ctrl.getCurrentMotionState(&state)) {
            std::cout << "运动状态: " << (int)state << std::endl;
            if (state == 6) {
                std::cout << "已站立!" << std::endl;
                break;
            }
        }
    }

    // 坐下
    std::cout << "发送坐下指令..." << std::endl;
    ctrl.setSitDown();
    std::this_thread::sleep_for(std::chrono::seconds(3));

    ctrl.disconnect();
    std::cout << "测试完成" << std::endl;
    return 0;
}
