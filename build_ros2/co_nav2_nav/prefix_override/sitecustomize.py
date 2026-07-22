import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/isee-cdh/ws/Co-NavGPT2/install_ros2/co_nav2_nav'
