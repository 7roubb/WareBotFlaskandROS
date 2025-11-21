import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/super/jazzy_ws/src/warebot_robot_bridge/install/warebot_robot_bridge'
