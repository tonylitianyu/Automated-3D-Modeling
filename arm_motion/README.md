# Arm Motion Package

## Author: Blocked

This package contains the launchfiles and nodes to integrate the `camera_reconstruct` point cloud construction pipeline with the Sawyer robot and turtlebot3. This package provides the nodes `scanner_arm` and `turntable`.   

### `scanner_arm` node
The `scanner_arm` node calls a service provided by the `camera_reconstruct` package to save point clouds captured by a RealSense depth camera mounted on the end-effector of a Sawyer robot, and calls a service provided by the `turntable` node to control the turtlebot3's rotation.

### `turntable` node 
The `turntable` node controls the turtlebot3's rotation and provides a `timer` service to enable other nodes to interact with it.

## Usage Instructions
1. Download this package along with the other 2 packages included in this repository. 
2. Do `catkin_make` in workspace.
3. Source workspace `source devel/setup.bash`.
4. Connect to Sawyer robot and turtlebot (instructions in top level README).
5. Export turtlebot3 model:
```
export TURTLEBOT3_MODEL=burger
```
6. Run launch file:
```
roslaunch arm_motion arm.launch
```
