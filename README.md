# SP1_CREATE

CREATE Lab EPFL
Semester project : Enhanced Robot Navigation Through Multi-Sensor Integration
Goal : improve obstacle avoidance on a delivery robot
Autumn semester 2025

Instructions to run the code :

ssh <user-id>@<name>.local         # to connect to the Pi in SSH
. setup_robot.sh                   # init script
cd SP1_CREATE/ros_robot/           # to go to the right directory
source install/setup.bash          # to setup ROS
colcon build                       # to build if the code was changed
ros2 run robot fsm                 # to start the nodes (one in each terminal)
ros2 run robot display_node
ros2 run robot camera_node
