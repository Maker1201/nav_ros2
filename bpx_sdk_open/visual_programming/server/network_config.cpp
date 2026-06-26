#include "network_config.h"
#include "json.hpp"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>

#ifdef __linux__
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <linux/if.h>
#include <netdb.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace bpx_vp {

using json = nlohmann::json;

namespace {

uint32_t ipToUint(const std::string& ip) {
    struct in_addr addr;
    if (inet_aton(ip.c_str(), &addr) == 0) return 0;
    return ntohl(addr.s_addr);
}

}  // namespace

bool isSameSubnet(const std::string& ip1, const std::string& ip2, uint32_t netmask) {
    uint32_t a = ipToUint(ip1);
    uint32_t b = ipToUint(ip2);
    if (a == 0 || b == 0) return false;
    return (a & netmask) == (b & netmask);
}

std::string getInterfaceType(const std::string& ifname) {
#ifdef __linux__
    // 最可靠的方法：检查 phy80211 目录（无线接口独有）
    std::string phy_path = "/sys/class/net/" + ifname + "/phy80211";
    {
        struct stat st;
        if (stat(phy_path.c_str(), &st) == 0 && S_ISLNK(st.st_mode)) {
            return "wireless";
        }
    }

    // 检查 wireless 目录
    std::string wireless_path = "/sys/class/net/" + ifname + "/wireless";
    {
        struct stat st;
        if (stat(wireless_path.c_str(), &st) == 0 && S_ISDIR(st.st_mode)) {
            return "wireless";
        }
    }

    // 通过名称前缀判断（在 type 检查之前，因为 type=1 不可靠）
    if (ifname.size() >= 2 && ifname.substr(0, 2) == "wl") {
        return "wireless";
    }
    if (ifname.size() >= 3 && ifname.substr(0, 3) == "usb") {
        return "usb";
    }
    if (ifname.size() >= 3 && (ifname.substr(0, 3) == "eth" ||
        ifname.substr(0, 4) == "enp" || ifname.substr(0, 4) == "eno" ||
        ifname.substr(0, 4) == "ens")) {
        return "ethernet";
    }

    // 最后才用 /sys/class/net/<name>/type（不可靠，无线也返回 1）
    std::string path = "/sys/class/net/" + ifname + "/type";
    std::ifstream f(path);
    if (f.is_open()) {
        int type = 0;
        f >> type;
        if (type == 803) return "wireless";
        if (type == 1) return "ethernet";
    }
#endif
    return "other";
}

std::vector<NetworkInterface> enumerateInterfaces(const std::string& robot_ip) {
    std::vector<NetworkInterface> result;

#ifdef __linux__
    struct ifaddrs* ifaddr = nullptr;
    if (getifaddrs(&ifaddr) == -1) return result;

    for (struct ifaddrs* ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == nullptr) continue;
        if (ifa->ifa_addr->sa_family != AF_INET) continue;  // 仅 IPv4

        // 跳过 loopback
        if (ifa->ifa_flags & IFF_LOOPBACK) continue;

        NetworkInterface iface;
        iface.name = ifa->ifa_name;

        char ip_buf[INET_ADDRSTRLEN];
        struct sockaddr_in* addr_in = reinterpret_cast<struct sockaddr_in*>(ifa->ifa_addr);
        inet_ntop(AF_INET, &addr_in->sin_addr, ip_buf, INET_ADDRSTRLEN);
        iface.ip = ip_buf;

        // 获取实际子网掩码
        uint32_t netmask = 0xFFFFFF00;  // 默认 /24
        if (ifa->ifa_netmask != nullptr) {
            struct sockaddr_in* mask_in = reinterpret_cast<struct sockaddr_in*>(ifa->ifa_netmask);
            netmask = ntohl(mask_in->sin_addr.s_addr);
        }

        iface.is_up = (ifa->ifa_flags & IFF_UP) != 0;
        iface.type = getInterfaceType(iface.name);
        iface.same_subnet_as_robot = isSameSubnet(iface.ip, robot_ip, netmask);

        // 获取 MAC 地址
        std::string mac_path = "/sys/class/net/" + iface.name + "/address";
        std::ifstream mac_file(mac_path);
        if (mac_file.is_open()) {
            std::getline(mac_file, iface.mac);
        }

        result.push_back(iface);
    }

    freeifaddrs(ifaddr);
#endif

    // 按优先级排序：同网段 > 已启用 > 有线 > 无线 > 其他
    std::sort(result.begin(), result.end(), [](const NetworkInterface& a, const NetworkInterface& b) {
        if (a.same_subnet_as_robot != b.same_subnet_as_robot)
            return a.same_subnet_as_robot > b.same_subnet_as_robot;
        if (a.is_up != b.is_up)
            return a.is_up > b.is_up;
        auto type_priority = [](const std::string& t) -> int {
            if (t == "ethernet") return 0;
            if (t == "wireless") return 1;
            if (t == "usb") return 2;
            return 3;
        };
        return type_priority(a.type) < type_priority(b.type);
    });

    return result;
}

std::string findMatchingInterface(const std::string& robot_ip) {
    auto ifaces = enumerateInterfaces(robot_ip);
    for (const auto& iface : ifaces) {
        if (iface.same_subnet_as_robot && iface.is_up) {
            return iface.name;
        }
    }
    // 没有同网段的，返回第一个已启用的
    for (const auto& iface : ifaces) {
        if (iface.is_up) {
            return iface.name;
        }
    }
    return "";
}

NetworkConfig parseArgs(int argc, char** argv) {
    NetworkConfig config;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if ((arg == "--interface" || arg == "-i") && i + 1 < argc) {
            config.local_interface = argv[++i];
        } else if ((arg == "--robot-ip" || arg == "-r") && i + 1 < argc) {
            config.robot_ip = argv[++i];
        } else if ((arg == "--web-port" || arg == "-w") && i + 1 < argc) {
            config.web_server_port = static_cast<uint16_t>(std::stoi(argv[++i]));
        } else if ((arg == "--state-port") && i + 1 < argc) {
            config.state_udp_port = static_cast<uint16_t>(std::stoi(argv[++i]));
        } else if ((arg == "--joint-port") && i + 1 < argc) {
            config.joint_state_udp_port = static_cast<uint16_t>(std::stoi(argv[++i]));
        } else if ((arg == "--tcp-port") && i + 1 < argc) {
            config.tcp_local_port = static_cast<uint16_t>(std::stoi(argv[++i]));
        } else if ((arg == "--state-rate") && i + 1 < argc) {
            config.state_upload_rate = static_cast<uint16_t>(std::stoi(argv[++i]));
        } else if ((arg == "--cmd-rate") && i + 1 < argc) {
            config.motion_command_rate = static_cast<uint16_t>(std::stoi(argv[++i]));
        } else if ((arg == "--config" || arg == "-c") && i + 1 < argc) {
            config = loadConfig(argv[++i]);
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "BPX Visual Programming Server\n\n"
                      << "Options:\n"
                      << "  -i, --interface <name>   Network interface (default: auto)\n"
                      << "  -r, --robot-ip <ip>      Robot IP address (default: 10.21.20.1)\n"
                      << "  -w, --web-port <port>    Web server port (default: 8080)\n"
                      << "  --state-port <port>      UDP state port (default: 9873)\n"
                      << "  --joint-port <port>      UDP joint state port (default: 7895)\n"
                      << "  --tcp-port <port>        TCP local port (default: 0=auto)\n"
                      << "  --state-rate <hz>        State upload rate (default: 100)\n"
                      << "  --cmd-rate <hz>          Motion command rate (default: 50)\n"
                      << "  -c, --config <path>      Load config from JSON file\n"
                      << "  -h, --help               Show this help\n";
            std::exit(0);
        }
    }

    return config;
}

NetworkConfig loadConfig(const std::string& path) {
    NetworkConfig config;
    try {
        std::ifstream f(path);
        if (!f.is_open()) {
            std::cerr << "Cannot open config file: " << path << std::endl;
            return config;
        }
        json j = json::parse(f);
        if (j.contains("robot_ip")) config.robot_ip = j["robot_ip"].get<std::string>();
        if (j.contains("local_interface")) config.local_interface = j["local_interface"].get<std::string>();
        if (j.contains("state_udp_port")) config.state_udp_port = j["state_udp_port"].get<uint16_t>();
        if (j.contains("joint_state_udp_port")) config.joint_state_udp_port = j["joint_state_udp_port"].get<uint16_t>();
        if (j.contains("tcp_local_port")) config.tcp_local_port = j["tcp_local_port"].get<uint16_t>();
        if (j.contains("state_upload_rate")) config.state_upload_rate = j["state_upload_rate"].get<uint16_t>();
        if (j.contains("motion_command_rate")) config.motion_command_rate = j["motion_command_rate"].get<uint16_t>();
        if (j.contains("web_server_port")) config.web_server_port = j["web_server_port"].get<uint16_t>();
        if (j.contains("web_server_bind")) config.web_server_bind = j["web_server_bind"].get<std::string>();
    } catch (const std::exception& e) {
        std::cerr << "Failed to parse config file: " << e.what() << std::endl;
    }
    return config;
}

bool checkRobotConnectivity(const std::string& robot_ip, int timeout_ms) {
#ifdef __linux__
    std::string cmd = "ping -c 1 -W " + std::to_string(timeout_ms / 1000) + " " + robot_ip + " > /dev/null 2>&1";
    return system(cmd.c_str()) == 0;
#else
    return false;
#endif
}

}  // namespace bpx_vp
