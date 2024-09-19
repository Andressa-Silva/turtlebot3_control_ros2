import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/andressa/turtlebot3_ws/src/turtlebot3_control_ros2/install/turtlebot3_control_ros2'
