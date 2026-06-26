import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/dog/nav_ros/install/bpx_ekf_odom_ros'
