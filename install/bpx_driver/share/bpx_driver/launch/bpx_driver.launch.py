import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('bpx_driver'),
        'config',
        'bpx_params.yaml'
    )

    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip', default_value='10.21.20.1',
        description='BPX robot IP address (Wi-Fi: 192.168.0.1 or 10.38.160.85, wired: 10.21.40.1)')

    publish_odom_tf_arg = DeclareLaunchArgument(
        'publish_odom_tf', default_value='false',
        description='Publish odom->base_link TF (disable when using EKF)')

    urdf_path = os.path.join(
        get_package_share_directory('bpx_driver'),
        'bpx', 'urdf', 'bpx.urdf'
    )
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    bpx_driver_node = Node(
        package='bpx_driver',
        executable='bpx_driver_node',
        name='bpx_driver_node',
        parameters=[config, {
            'robot_ip': LaunchConfiguration('robot_ip'),
            'publish_odom_tf': LaunchConfiguration('publish_odom_tf'),
        }],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        robot_ip_arg,
        publish_odom_tf_arg,
        robot_state_pub,
        bpx_driver_node,
    ])
