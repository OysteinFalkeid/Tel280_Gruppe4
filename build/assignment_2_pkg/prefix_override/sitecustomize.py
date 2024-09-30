import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/Gruppe4/ros2_ws/install/assignment_2_pkg'
