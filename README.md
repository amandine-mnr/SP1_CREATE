# SP1_CREATE

**CREATE Lab EPFL**  
**Semester project :** Enhanced Robot Navigation Through Multi-Sensor Integration  
**Goal :** improve obstacle avoidance on a delivery robot  
Autumn semester 2025

Instructions to run the code :

```
ssh <user-id>@<name>.local         # connect to the Pi in SSH  
. setup_robot.sh                   # init script  
cd SP1_CREATE/ros_robot/           # navigate to ROS workspace  
colcon build                       # build if needed

# Run each node in a separate terminal   
ros2 run robot fsm 
ros2 run robot display_node  
ros2 run robot camera_node
```
