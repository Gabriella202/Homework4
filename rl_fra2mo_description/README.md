# Robotics Lab Mobile Robot

This repo provides URDF files and launch files to simulate a differential drive robot

```bash
colcon build --packages-select rl_fra2mo_description
. install/setup.bash
  ``` 
## Visualize on Rviz
```bash
ros2 launch rl_fra2mo_description display_fra2mo.launch.py
  ```
  
## Start Gazebo simulation
```bash
ros2 launch rl_fra2mo_description gazebo_fra2mo.launch.py
  ```
  
## Send velocity command
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.2}}"
  ```
  
## Launch SLAM to map the environment (autonomous navigation not included)
```bash
ros2 launch rl_fra2mo_description fra2mo_slam.launch.py
  ```
  
## Sending velocity commands using keybord
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
  ```
  
## Save map
Create a ```maps``` folder and add it to the ```CMakeLists.txt```, inside the folder run
```bash
ros2 run nav2_map_server map_saver_cli -f map
  ```
  
## Launch AMCL to see how localization works in the mapped environment (autonomous navigation not included)
```bash
ros2 launch rl_fra2mo_description fra2mo_amcl.launch.py
  ```

## Autonomous navigation with AMCL
```bash
ros2 launch rl_fra2mo_description fra2mo_navigation.launch.py
  ```
To give Nav2 goal poses and visualize what is happening open Rviz with proper configuration

## Autonomous exploration and mapping of the environment
```bash
git clone https://github.com/robo-friends/m-explore-ros2.git
colcon build --packages-select explore_lite
. install/setup.bash
ros2 launch rl_fra2mo_description fra2mo_explore.launch.py
  ```
To visualize the mapping open Rviz with proper configuration
