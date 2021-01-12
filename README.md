# #Blocked
Authors: Tianyu Li, Arun Kumar, Bowen Feng, Nicole Baptist, Yen Chin Loke

<br />

# Overview

## Introduction
This project creates an automated system for reconstructing objects with point clouds. The hardware for this system includes a Rethink Sawyer robot arm and a RealSense camera as the end-effector, and a turtlebot3 (burger).

The turtlebot3 is used as a turntable to allow the RealSense camera to record point cloud data for the object from all sides. The Sawyer robot arm is used to control the position and the viewing angle of the camera. 

In the current version, our system will record point clouds from four sides and the top view of the object to recontruct its point cloud.  

## My Contribution
- Came up with the idea and design the approach
- Implemented the turntable motion for turtlebot using timer open-loop control
- Enhanced the turtlebot turntable motion precision and accuracy with ```slam_toolbox```  and  ```move_base```
- Created the camera_motion package for alignment at the initial setup with ```OpenCV```
- Fused and cropped the captured point clouds using ```C++``` with ```PCL```
- Generated mesh for the point clouds using  ```Open3D library``` and output it to .ply file

## Demo
![](camera_reconstruct/videos/slam_base.gif)

Full demo in lab with Sawyer: [Video](https://drive.google.com/file/d/1BcSyo_AOadVpFWu_8wsWRbMq8CBYiBKo/view?usp=sharing)

Final demo at home: [Video](https://drive.google.com/file/d/1Ju85MJpGs4Qd3p4OVBtpFYB5iKh3aYQW/view?usp=sharing)
<br />

## File structure
```
--final-project-blocked
  --arm_motion
  --camera_motion
  --camera_reconstruct
```

## Quickstart Guide
### Setup the workspace
```
mkdir -p ws/src
cd ws/src
wstool init .
wstool merge https://github.com/ME495-EmbeddedSystems/final-project-blocked/blob/master/blocked.rosinstall
wstool update 
cd ..
catkin_make
```
OR install the packages listed in [Packages and Dependencies](#packages-and-dependencies) then:
1. Clone this repository into your workspace
```
git clone https://github.com/ME495-EmbeddedSystems/final-project-blocked
```
2. Install: do in workspace `catkin_make`
3. Source workspace `source devel/setup.bash`

### Operating Instructions
1. Power on Sawyer and turtlebot3. Ensure that the Sawyer, turtlebot3, and your computer are running on the same `ROS_MASTER_URI` (by connecting Sawyer and computer by ethernet to a router, and turtlebot3 connect to wifi).
2. Setup Sawyer 
    ```
    nmcli con up Rethink
    export ROS_MASTER_URI=http://10.42.0.2:11311
    export ROS_IP=10.42.0.1
    unset ROS_HOSTNAME
    rosrun intera_interface enable_robot.py -e
    rosrun intera_interface joint_trajectory_action_server.py
    ```
3. Setup turtlebot3
    - ssh to turtlebot3
    - launch turtlebot3 on turtblebot3
        ```
        roslaunch turtlebot3_bringup turtlebot3_robot.launch
        ```
4. Plug in RealSense to computer
5. Launch Project
    ```
    export TURTLEBOT3_MODEL=burger
    roslaunch arm_motion arm.launch
    ```
### Operating Instructions for improved version (with SLAM and move_base)

1. Power turtlebot3. Ensure that turtlebot3, and your computer are running on the same `ROS_MASTER_URI`.
2. Launch from remote computer
    ```
    roslaunch arm_motion home.launch
    ```
3. Setup turtlebot3
    - update turtlebot3 time
      ```
      ssh ubuntu@turtlebot.local sudo date -s @`(date -u +"%s")`
      ```
    - ssh to turtlebot3
    - launch turtlebot3 on turtblebot3
      ```
      roslaunch turtlebot3_bringup turtlebot3_robot.launch
      ```
 4. In rviz, make sure the Global Fixed Frame is ```camera_depth_optical_frame``` and the PointCloud2 topic is ```/camera/depth/color/points```.
 5. Adjust the actual camera position. Put the target pointcloud showing in rviz at the center of the ```object``` tf frame.
 6. Call service on remote computer to start the scan
    ```
    rosservice call /rotate
    ```
      
      

## Packages and Dependencies
This package assumes the user has packages from the custom `nuws` and `rethink_ws` installed (ME495 Fall 2020 course website).   
To install the additional required packages:
```
sudo apt install ros-noetic-tf2-sensor-msgs
sudo apt install ros-noetic-pcl-ros
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

## Overall System Architecture
This project contains three packages that are used together to complete the 3D scanning task
### `camera_reconstruct` package 
This package provides a pipeline for saving multiple point clouds and fuse them into a single point cloud. The package provides a service to save a point clouds and sends saved point clouds through pipeline. Click [here](#camera-reconstruct-package) for more details.
### `camera_motion` package 
This package contains a tool for aligning the center of the depth camera with the scanning object during initial setup. Click [here](#camera-motion-package) for more details.
### `arm_motion` package 
This package provides nodes to concurrently control the Sawyer robot and turtlebot3, and takes advantage of the pipeline from `camera_reconstruct` to generate the 3D scan form multiple camera views. Click [here](#arm-motion-package) for more details



## Camera Reconstruct Package

This package contains a pipeline to save multiple point clouds and fuse them into a single point cloud. The package provides a service to save a point clouds and sends saved point clouds through pipeline. This package is ideal for 3D scanning.


### Usage Instructions:
1. Add package to the src folder in your ROS workspace
2. Compile: `catkin_make`
3. Connect Real Sense Camera
4. Start simulation: `roslaunch camera_reconstruct viewer.launch gaz:=true`
5. Use `/generate_cp/save_pc` to save current point cloud, transform point cloud about TurtleBot base_scan frame, and crop point cloud
    * If the service is called multiple times, all point clouds will be fused and published to `/fused_pc` topic

### Configuration Instructions:
1. viewer.launch configuration
    * The viewer launch file contains arguements gaz and clip
    * Set clip true to use a camera clipping distance of 0.7
    * Set gaz to true to launch a point cloud viewer without a physical TurtleBot or Sawyer
        * The TurtleBot is launch in Gazebo and the Sawyer is not launched
        * Publish twist messages to /cmd_vel while calling /generate_pc/save_pc to test point_cloud fusion pipeline
    * Modify PointCloud2 topic in rviz to see live point cloud, last saved point cloud, or fused point cloud
    * Set slam to ture uses the slam_toolbox and move_base frame for reconstruction

### Saving Pointclouds:
1. Use `rosrun pcl_ros pointcloud_to_pcd input:=/fused_pc` to save a .pcd file of the fused pointcloud to the current directory
2. .pcd files can be viewed in pcl_viewer by using the command `pcl_viewer -multiview 1 path_to_.pcd `

### Nodes
1. `generate_pc`
    * Subscribes to `PointCloud2` messages from Realsense camera
    * Provides a service to transform and publish current pointcloud to `/saved_pcs` topic
2. `generate_pc_sim`
    * Same functionality as `generate_pc` but works in simulation without the Sawyer robot
    * Used for at home testing
3. `generate_pc_slam`
    * Same functionality as `generate_pc` but works with the SLAM implementation of turtlebot3
    * Used in home testing
4. `fuse_pc`
    * Subscribes to `/saved_pcs` topic
    * Concatenates all saved_pcs into a single PointCloud2 message
    * Crops fused pointcloud
    * Publishes resulting pointcloud to `/fused_pc`
5. `fuse_pc_slam`
    * Same function as `fuse_pc` but works with SLAM implementation of turtlebot3
    * Used for home testing
6. `test_pointcloud`
    * Test file to ensure `save_pc` service is functional
    
### Future Work
#### ICP (Iterative Closest Point)
ICP can find the transformation between two point clouds. It will be helpful for fusing point clouds.

<br />

## Camera Motion Package

This package contains a tool for aligning the center of the depth camera with the scanning object during initial setup.

### `find_depth` node
The `find_depth` node publishes the depth value of the center <b>n x n</b> pixel. In this way, one can know that if the object is at a the center of the cameraview by checking if the published depth value is at the required distance.

### `camera_motion` python library
This library contains function for calculating the average depth value across space and time to obtain a more stable depth value. For space, it is taking the average depth values of the <b>n x n</b> center pixels. For time, it is taking the average depth values across the past <b>m</b> seconds. This library also comes with unit testing.

### Usage Instructions:
Rosrun it during the initial hardware setup
 ```
 rosrun camera_motion find_depth
 ```

<br />

## Arm Motion Package

This package contains the launchfiles and nodes to integrate the `camera_reconstruct` point cloud construction pipeline with the Sawyer robot and turtlebot3. This package provides the nodes `scanner_arm` and `turntable`.   

### Nodes

1. `scanner_arm` node   
The `scanner_arm` node calls a service provided by the `camera_reconstruct` package to save point clouds captured by a RealSense depth camera mounted on the end-effector of a Sawyer robot, and calls a service provided by the `turntable` node to control the turtlebot3's rotation.

2. `turntable` node    
The `turntable` node controls the turtlebot3's rotation and provides a `timer` service to enable other nodes to interact with it.

3. `turtle` node 
The `turtle` node implements SLAM on the turtlebot3 to get improved tf feedback for the fusing of pointclouds

### Launch files 
1. `arm.launch`    
Overall launch file to launch entire system from this project
2. `turtlebot.launch`
Launches the `turntable` node and a `robot_state_publisher`
3. `home.launch`
Motion with slam_toolbox and move_base
4. `testbots.launch`
For testing loading 2 robots into gazebo at the same time

### Future Work
#### Scanning from multiple angles
This was a stretch goal that we did not manage to accomplish. We wrote the `scanner_arm_multi` and `turntable_multi` nodes to allow for multiple angle scans by the camera on the sawyer end_effector, but they are currently not in use for this version of the project. The diference between this "multi" implementation and the current implementation is the way we scan the image. In this stretch goal, we scan the image in the top, middle and bottom three degrees. After the turtlebot3 turns 90 degrees, the `turntable_multi` node publishes `"sawyer"` on the `/next` topic to call the Sawyer to move through three fixed waypoints for capturing point clouds. At each waypoint, `scanner_arm_multi` calls the `/generate_pc/save_pc` service to save the point cloud. After completing this series of motions, Sawyer publishes `"turtle"` on the `/next` topic to turn the turtlebot3 turn 90 degrees again using the `turntable_multi` node. Preliminary versions of the nodes for this motion is done but we didn't have time to test and refine it becasue the 3D reconstruction of this is complicated. 
