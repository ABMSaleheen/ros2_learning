import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/saleheen_linux/others/ros_2_Learning/ros2_workspace/install/prius_line_following'
