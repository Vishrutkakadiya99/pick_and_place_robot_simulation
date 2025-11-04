import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/arjun/vishrut/robot-arm-pick-and-place/install/robot_control'
