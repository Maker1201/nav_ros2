import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = 'nav2'
    pkg_dir = get_package_share_directory(pkg_name)
    
    # 指向新的纯定位 lua 文件
    cartographer_config_dir = os.path.join(pkg_dir, 'config')
    cartographer_config_basename = 'my_localization.lua'
    
    # 刚才保存的 pbstream 地图文件的绝对路径！
    pbstream_file_path = '/home/dog/nav_ros/src/nav2/maps/my_map.pbstream'

    return LaunchDescription([
        # 1. 启动 rf2o 激光里程计 (必须有它提供 odom->base_link)
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            output='screen',
            parameters=[{
                'laser_scan_topic' : '/scan',
                'odom_topic' : '/odom',
                'publish_tf' : True,
                'base_frame_id' : 'base_link',
                'odom_frame_id' : 'odom',
                'init_pose_from_topic' : '',
                'freq' : 20.0
            }],
        ),

        # 2. 启动 Cartographer (纯定位模式)
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', cartographer_config_basename,
                '-load_state_filename', pbstream_file_path  # 【核心改变】加载静态地图文件！
            ],
            remappings=[
                ('scan', '/scan'),
                ('odom', '/odom')
            ]
        ),

        # 3. 静态 TF 发布器 (必须保留，告诉 Cartographer 雷达在哪)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_laser',
            arguments=['0.0', '0.0', '0.2', '0.0', '0.0', '0.0', 'base_link', 'laser']
        ),
        
        # 注意：这里我们去掉了 cartographer_occupancy_grid_node
        # 因为在标准导航中，.yaml 静态地图将由 Nav2 的 Map Server 来发布。
    ])