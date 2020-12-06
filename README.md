# #Blocked
This is the final group project for ME495 Fall 2020 at Northwestern University  
Authors: Arun Kumar, Bowen Feng, Nicole Baptist, Tianyu Li, Yen Chin Loke

## Introduction
This project creates an automated system for reconstructing objects with point clouds. The hardware for this system includes a Rethink Sawyer robot arm and a RealSense camera as the end-effector, and a turtlebot3 (burger).

The turtlebot3 is used as a turntable to allow the RealSense camera to record point cloud data for the object. The Sawyer robot arm is used to control the position and the viewing angle of the camera. 

In the current version, our system will record point cloud from four sides and the top view of the object to recontruct its point cloud.  

## File structure:
```
--final-project-blocked
  --arm_motion
  --camera_motion
  --camera_reconstruct
```
## Packages & Dependencies
This package assumes the user has packages from the custom `nuws` and `rethink_ws` installed (ME495 Fall 2020 course website).   
To install the additional required packages:
```
sudo apt install python3-pcl
```
To install sawyer_moveit, do:
```
cd ~/rethink_ws/src
wstool init .
wstool merge https://raw.githubusercontent.com/RethinkRobotics/sawyer_moveit/master/sawyer_moveit.rosinstall
wstool update
cd ..
catkin_make
```

## Workspace Setup Instructions
1. Clone this repository into your workspace. 
2. Install: do in workspace `catkin_make`
3. Source workspace `source devel/setup.bash`

## Operating Instructions
1. Power on Sawyer and Turtlebot
2. Setup Sawyer 
    ```
    nmcli con up Rethink
    export ROS_MASTER_URI=http://10.42.0.2:11311
    export ROS_IP=10.42.0.1
    unset ROS_HOSTNAME
    rosrun intera_interface enable_robot.py -e
    rosrun intera_interface joint_trajectory_action_server.py
    ```
3. Setup Turtlebot
    - ssh to turtlebot3
    - launch turtlebot3 on turtblebot3
        ```
        roslaunch turtlebot3_bringup turtlebot3_robot.launch
        ```
4. Plug in RealSense
5. Launch Project
    ```
    export TURTLEBOT3_MODEL=burger
    roslaunch arm_motion arm.launch
    ```


    
