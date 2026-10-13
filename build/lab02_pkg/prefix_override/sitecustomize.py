import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/moradlinux/ros2_lab_workspace/install/lab02_pkg'
