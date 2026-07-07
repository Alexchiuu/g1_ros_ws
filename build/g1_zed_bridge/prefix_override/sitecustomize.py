import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/mnt/home/calex/g1_ros_ws/install/g1_zed_bridge'
