import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = 'nav2'
    cartographer_config_dir = os.path.join(get_package_share_directory(pkg_name), 'config')
    cartographer_config_basename = 'my_cartographer_2d.lua'

    return LaunchDescription([
        # 1. 激光里程计
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

        # 2. Cartographer 核心节点
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', cartographer_config_basename
            ],
            remappings=[
                ('scan', '/scan'),
                ('odom', '/odom')
            ]
        ),

        # 3. 栅格地图节点
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            arguments=[
                '-resolution', '0.05',
                '-publish_period_sec', '1.0'
            ]
        ),

        # 4. 【新增】静态 TF 发布器：连接底盘与雷达！
        # 假设雷达装在底盘正中心上方 0.2 米处。这里的 'laser' 完美对应你报错里缺失的 source_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_laser',
            arguments=['0.0', '0.0', '0.2', '0.0', '0.0', '0.0', 'base_link', 'laser']
        ),
    ])
    