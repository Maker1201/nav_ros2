# BPX SDK 部署指南


---

## 1. 环境准备（目标电脑）

```bash
# Ubuntu 22.04+ 需要安装的工具
sudo apt update
sudo apt install -y build-essential cmake
```

只需要这两样：**C++17 编译器** 和 **CMake 3.10+**。没有其他依赖。

---

## 2. 拷贝项目

整个项目目录拷贝过去即可，**不需要 `build/` 目录**（目标机上重新编译）：

---

## 3. 编译 SDK 示例

```bash
cd /home/user/bpx_sdk_open
mkdir build && cd build
cmake ..
make
```

CMake 会自动检测目标电脑的 CPU 架构（x86_64 或 aarch64），链接对应的 `.so` 文件。

---

## 4. 编译图形化编程系统

```bash
cd /home/user/bpx_sdk_open/visual_programming
mkdir build && cd build
cmake ..
make
```

编译完成后，`build/` 目录下会生成可执行文件，并自动拷贝 `frontend/` 前端资源。

---

## 5. 网络配置（连接机器人）

目标电脑需要能访问 BPX 机器人的 IP 地址：

| 连接方式 | 机器人 IP | 说明 |
|---------|----------|------|
| 有线以太网 | `10.21.20.1` | 默认方式 |
| WiFi | `192.168.0.1` | 无线连接 |
| USB 网络共享 | 自动分配 | USB 线直连 |

如果机器人 IP 不是默认值，可以在代码中修改 `include/bpx_sdk_config.h`，或在运行时通过 `setRobotIp()` 设置。

---

## 6. 运行

```bash
# 运行图形化编程服务器
cd /home/user/bpx_sdk_open/build
  ./bpx_visual_programming

# 然后浏览器访问 http://localhost:8080
```

---

## 常见问题

| 问题 | 解决方案 |
|------|---------|
| `libbpx_sdk_x86_64.so: cannot open shared object file` | 运行 `export LD_LIBRARY_PATH=/home/user/bpx_sdk_open/lib:$LD_LIBRARY_PATH`，或在 CMakeLists 中已处理 |
| CMake 版本太低 | `sudo apt install cmake` 或从官网下载新版 |
| 目标电脑是 ARM 架构（如树莓派） | 没问题，项目自带 `libbpx_sdk_aarch64.so`，CMake 自动选择 |
| 想改机器人 IP 但不想改代码 | 运行时传参：`./bpx_visual_programming --robot-ip 192.168.0.1` |

---

