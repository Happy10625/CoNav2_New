import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/isee-cdh/ws/Co-NavGPT2/install_normal/co_nav2_nav'
