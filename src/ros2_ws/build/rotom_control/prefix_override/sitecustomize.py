import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/caden/Documents/rotom/src/ros2_ws/install/rotom_control'
