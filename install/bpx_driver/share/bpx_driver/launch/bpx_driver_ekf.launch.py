import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
        description='BPX robot IP address (Wi-Fi: 192.168.0.1 or 10.38.160.85, wired: 10.21.20.1)')

    publish_odom_tf_arg = DeclareLaunchArgument(
        'publish_odom_tf', default_value='false',
        description='Publish odom->base_link TF (disable when using EKF)')

    use_ekf_odom_arg = DeclareLaunchArgument(
        'use_ekf_odom', default_value='true',
        description='Use EKF odometry (publish raw data for EKF fusion)')

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
            'use_ekf_odom': LaunchConfiguration('use_ekf_odom'),
        }],
        output='screen',
        emulate_tty=True,
    )

    # EKF odometry node
    ekf_odom_node = Node(
        package='bpx_ekf_odom_ros',
        executable='ekf_odom_node',
        name='ekf_odom_node',
        output='screen',
        parameters=[{
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'publish_tf': True,
            'publish_rate': 100.0,
            'wsum_min': 0.05,
            'height_lock': True,
            'R_pz': 0.04,
            'pz_ref': 0.0,
            'use_sqrt': True,
            'chi2_soft': 50.0,
            'repeat_R_factor': 4.0,
        }],
        remappings=[
            ('/bpx/imu/data_raw', '/bpx/imu/data_raw'),
            ('/bpx/joint_states', '/bpx/joint_states'),
        ],
    )

    return LaunchDescription([
        robot_ip_arg,
        publish_odom_tf_arg,
        use_ekf_odom_arg,
        robot_state_pub,
        bpx_driver_node,
        ekf_odom_node,
    ])
