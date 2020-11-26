# Camera Reconstruct Package
### Author: Blocked

#### This package contains a pipeline to save multiple point clouds and fuse them into a single point cloud. The package provides a service to save a point clouds and sends saved point clouds through pipeline. This package is ideal for 3D scanning.

#### Usage Instructions:
1. Add package to the src folder in your ROS workspace
1. Compile: `catkin_make`
1. Connect Real Sense Camera
1. Start simulation: `roslaunch camera_reconstruct viewer.launch gaz:=true`
1. Use /generate_cp/save_pc to save current point cloud, transform point cloud about TurtleBot base_scan frame, and crop point cloud
    * If the service is called multiple times, all point clouds will be fused and published to /fused_pc topic

#### Configuration Instructions:
1. viewer.launch configuration
    * The viewer launch file contains arguements gaz and clip
    * Set clip true to use a camera clipping distance of 0.7
    * Set gaz to true to launch a point cloud viewer without a physical TurtleBot or Sawyer
        * The TurtleBot is launch in Gazebo and the Sawyer is not launched
        * Publish twist messages to /cmd_vel while calling /generate_pc/save_pc to test point_cloud fusion pipeline
    * Modify PointCloud2 topic in rviz to see live point cloud, last saved point cloud, or fused point cloud
