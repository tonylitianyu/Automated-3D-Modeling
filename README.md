# final-project-blocked
This is final group project for ME495 at Northwestern University  
Author: Arun Kumar, Bowen Feng, Nicole Baptist, Tianyu Li, Yen Chin Loke

## Introduction
This project creates an automated system for reconstructing object with point clouds. The hardware for this system includes a Rethink Sawyer robot arm with a RealSense camera as the end-effector and a turtlebot (burger).

The turtlebot is used as a turn table to allow the realsense camera to record point cloud data for the object. The Sawyer robot arm is used to control the position and the pointing angle of the camera. 

For current version, our system will record point cloud from four sides and the top view of the object to recontruct its point cloud.  

## File structure:
```
--final-project-blocked
  --arm_motion
  --camera_motion
  --camera_reconstruct
```
## Packages & Dependencies

    ```
    sudo apt install python3-pcl
    ```

## Instruction
1. Turn on Sawyer and Turtlebot
2. Setup Sawyer 
    ```
    nmcli con up Rethink
    export ROS_MASTER_URI=http://10.42.0.2:11311
    export ROS_IP=10.42.0.1
    unset ROS_HOSTNAME
    source rethink_ws/devel/setup.bash
    rosrun intera_interface enable_robot.py -e
    rosrun intera_interface joint_trajectory_action_server.py
    ```
3. Setup Turtlebot
    - ssh to turtlebot
    - launch turtlebot on turtblebot
        ```
        roslaunch turtlebot3_bringup turtlebot3_robot.launch
        ```

4. Launch Project
    ```
    roslaunch arm_motion arm.launch
    ```


    
