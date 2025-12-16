import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/guerinel/Documents/info_rob/ROS2/TP3_controle_robotique_turtlebot/install/turtlebot_controller'
