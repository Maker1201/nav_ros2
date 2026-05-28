include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  
  -- 坐标系配置
  map_frame = "map",
  tracking_frame = "base_link",       
  published_frame = "odom",           
  odom_frame = "odom",                
  
  -- 运行模式
  provide_odom_frame = false,         
  publish_frame_projected_to_2d = true,
  
  -- 传感器利用
  use_odometry = false,                
  use_nav_sat = false,
  use_landmarks = false,
  
  num_laser_scans = 1,                
  num_multi_echo_laser_scans = 0,     
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,

  -- =====================================
  -- 修复点：添加缺失的采样率参数 (Humble 版本强制要求)
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
  -- =====================================
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- 轨迹构建器核心参数
TRAJECTORY_BUILDER_2D.use_imu_data = false  
TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 12.0      
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0

-- 开启在线相关性扫描匹配（CSM）
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

return options