# ROS 2 Dynamic Resource Allocation Multi Robot System
A simple 2D multi-AGVs simulator with `Rviz2` on ROS 2 `humble`, based on `open_rmf`

Required packages:
- [`open-rmf`] (https://github.com/open-rmf/rmf)

# Launching the Simulation
This is to start Gazebo rendering
```bash
 ros2 launch ign_sim v2.launch.xml
```
Custom path and robot marker display on Rviz
```bash
 ros2 launch dram_viz dram.viz.launch.py
```
Start DRAM modules
```bash
 ros2 launch dram_viz robots.launch.py
```
To send a service to robot
```bash
 ros2 service call /move_robot dram_interface/srv/MoveRobot "{robot_name: 'AGV1', goal_vertex_name: 'AGV1'}"
```