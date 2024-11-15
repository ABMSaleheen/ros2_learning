import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/saleheen_linux/others/ros_2_Learning/ros2_ws_cpp/install/warehouse_rover_py'
