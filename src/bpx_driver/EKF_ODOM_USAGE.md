# EKF 里程计使用指南

## 启动方式
1. 使用 EKF 里程计: ros2 launch bpx_driver bpx_driver_ekf.launch.py
2. 使用简单里程计: ros2 launch bpx_driver bpx_driver.launch.py use_ekf_odom:=false

## 控制机器人
站立: ros2 service call /bpx/stand_up std_srvs/srv/Trigger
坐下: ros2 service call /bpx/sit_down std_srvs/srv/Trigger
