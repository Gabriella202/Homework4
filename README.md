# Robotics Lab 2024 Homework 4

This repo provides URDF files and launch files to simulate a mobile robot

To build all the packages :

```bash
colcon build 
  ``` 

After the build, before launching any commands, on every new open terminal, launch :

```bash
source install/setup.bash
  ``` 
  
## 1. Start Gazebo simulation

To visualize :
1. The robot at the desired position x = −3 m, y = 3.5 m, Y = −90 deg
2. The obstacle 9 at position x = −3 m, y = −3.3 m, z = 0.1 m, Y = 90 deg
3. The Aruco marker 115 positionated on the obstacle 9
launch :

```bash
ros2 launch rl_fra2mo_description gazebo_fra2mo.launch.py
  ```

To see the aruco marker, open the camera, and use the topic /camera :

```bash
ros2 run rqt_image_view rqt_image_view
  ```
  
## 2. Use Nav2 Commander

To launch the navigation command, first launch on th 1st terminal :
(Remember to push the play button before launching the other node)

```bash
ros2 launch rl_fra2mo_description centermap.launch.py 
  ```

Then, launch on a 2nd terminal the explore node :

```bash
ros2 launch rl_fra2mo_description fra2mo_explore.launch.py
  ```

Finally on the 3rd terminal, run the navigation :

```bash
ros2 run rl_fra2mo_description follow_waypoints.py
  ```
  
## 3. Map the enviroment

First thing first launch the centermap and fra2mo_explore launches as in the previous case, then run on the 3rd terminal :

a. To map the enviroment :

```bash
ros2 run rl_fra2mo_description tomapping.py
  ```

On Rviz2 it's possible to see the mapping (Rviz2 will automatically open with the right configuration)

b. To prove 4 different configurations, run instead :

- The conservative config :

```bash
ros2 run rl_fra2mo_description testing.py 1
  ```

- The aggressive config :

```bash
ros2 run rl_fra2mo_description testing.py 2
  ```

- The balanced config :

```bash
ros2 run rl_fra2mo_description testing.py 3
  ```

- The performing config :

```bash
ros2 run rl_fra2mo_description testing.py 4
  ```

## 4. Vision-based navigation

To open the Gazebo world with both the navigation and aruco node, launch :

```bash
ros2 launch rl_fra2mo_description vision_based.launch.py
  ```
  
b. To send the robot in proximity of the obstacle 9, detect the aruco and return to its initial position, run :

```bash
ros2 run rl_fra2mo_description detection.py
  ```

The pose of the Aruco will be automatically published when the marker is detected

c. To publish the aruco pose through the static TF, run on another terminal :

```bash
ros2 run rl_fra2mo_description static_aruco_tf2_broadcaster
  ```
