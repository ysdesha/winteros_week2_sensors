# WinteROS Week 2
# Part-2 : GAZEBO SENSORS
 <!--
## This is how far we will get by the end of this lesson: 
  <a href="https://youtu.be/0Xokl5dHRoQ"><img width="600" src="./assets/youtube-gazebo.png"></a>  

  <a href="https://youtu.be/ELwRqeNR_NA"><img width="600" src="./assets/youtube-gazebo-1.png"></a>  
-->

# Table of Contents
1. [Introduction](#introduction)  
1.1. [Download ROS package](#download-ros-package)  
1.2. [Test the starter package](#test-the-starter-package)  
2. [Camera](#camera)  
2.1. [Image transport](#image-transport)  
2.2. [rqt reconfigure](#rqt-reconfigure)  
2.3. [Wide angle camera](#wide-angle-camera)  
3. [Lidar](#lidar)  
3.1. [3D lidar](#3d-lidar) 
6. [RGBD camera](#rgbd-camera)  

# Introduction

In the first part of this week, you built a simulated robot that you can drive around manually. Now, it's time to verify that our robot can actually **sense** the world.

A robot that cannot sense its environment is just a remote-controlled car. In this lesson, we will equip our robot with two critical sensors:

- **Camera (Visual Perception):** To see the world in color.  
- **LiDAR (Spatial Perception):** To detect obstacles and measure distances.

We will focus on the **integration pipeline**:

- Adding the physical links in **URDF**
- Configuring the **Gazebo plugins**
- Establishing the **ROS 2 Bridge**

By the end of this module, you will be able to visualize what the robot sees in **RViz2** in real-time.

## Download ROS package

To download the starter package clone the following git repo  to your colcon workspace:
```bash
wip
```
## Test the starter package

After we download the package from GitHub, let's rebuild the workspace and source the `install/setup.bash` file to make sure ROS and its tools are aware about the new package.

Then we can test the package with the usual launch file:
```bash
ros2 launch erc_gazebo_sensors spawn_robot.launch.py
```

And we can also start a teleop node:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```




