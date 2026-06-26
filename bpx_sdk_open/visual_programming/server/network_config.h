#ifndef BPX_VISUAL_PROGRAMMING_NETWORK_CONFIG_H_
#define BPX_VISUAL_PROGRAMMING_NETWORK_CONFIG_H_

#include <cstdint>
#include <string>
#include <vector>

namespace bpx_vp {

struct NetworkInterface {
    std::string name;           // "eth0" / "wlan0" / "usb0"
    std::string ip;             // "10.21.20.100"
    std::string mac;            // "aa:bb:cc:dd:ee:ff"
    std::string type;           // "ethernet" / "wireless" / "usb" / "other"
    bool is_up;                 // 是否已启用
    bool same_subnet_as_robot;  // 是否与机器人同网段
};

struct NetworkConfig {
    std::string robot_ip = "10.21.20.1";
    std::string local_interface = "auto";
    uint16_t state_udp_port = 9873;
    uint16_t joint_state_udp_port = 7895;
    uint16_t tcp_local_port = 0;
    uint16_t state_upload_rate = 100;
    uint16_t motion_command_rate = 50;
    uint16_t web_server_port = 8080;
    std::string web_server_bind = "0.0.0.0";
};

// 枚举本机所有网络接口
std::vector<NetworkInterface> enumerateInterfaces(const std::string& robot_ip);

// 获取与指定 IP 同网段的接口名
std::string findMatchingInterface(const std::string& robot_ip);

// 判断两个 IP 是否在同一子网
bool isSameSubnet(const std::string& ip1, const std::string& ip2, uint32_t netmask = 0xFFFFFF00);

// 获取接口类型
std::string getInterfaceType(const std::string& ifname);

// 解析命令行参数
NetworkConfig parseArgs(int argc, char** argv);

// 从 JSON 配置文件加载
NetworkConfig loadConfig(const std::string& path);

// 检测机器人连通性（ping）
bool checkRobotConnectivity(const std::string& robot_ip, int timeout_ms = 2000);

}  // namespace bpx_vp

#endif  // BPX_VISUAL_PROGRAMMING_NETWORK_CONFIG_H_
