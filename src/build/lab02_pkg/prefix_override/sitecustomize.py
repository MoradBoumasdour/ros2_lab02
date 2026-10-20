import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/leonardo/ros2_lab_workspace/src/install/lab02_pkg'
