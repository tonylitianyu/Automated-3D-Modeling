# Camera Reconstruct Package
### Author: Blocked

#### This package contains a pipeline to save multiple point clouds and fuse them into a single point cloud. The package provides a service to save a point clouds and sends saved point clouds through pipeline. This package is ideal for 3D scanning.

#### Usage Instructions:
1. Add package to the src folder in your ROS workspace
2. Compile: `catkin_make`
3. Connect Real Sense Camera
4. Run the pointcloud_to_pcd node to subscribe to the /final_pc topic: `rosrun pcl_ros pointcloud_to_pcd input:=final_pc`
   
   //(Note: you can also specify the filename and save location by editing the necessary parameters: `rosrun pcl_ros pointcloud_to_pcd input:=final_pc prefix:=~/wsname filename:=nameurfile`)**//
5. Start simulation: `roslaunch camera_reconstruct viewer.launch gaz:=true`
6. Use /generate_cp/save_pc to save current point cloud, transform point cloud about TurtleBot base_scan frame, and crop point cloud
    * If the service is called multiple times, all point clouds will be fused and published to /fused_pc topic

#### Configuration Instructions:
1. viewer.launch configuration
    * The viewer launch file contains arguements gaz and clip
    * Set clip true to use a camera clipping distance of 0.7
    * Set gaz to true to launch a point cloud viewer without a physical TurtleBot or Sawyer
        * The TurtleBot is launch in Gazebo and the Sawyer is not launched
        * Publish twist messages to /cmd_vel while calling /generate_pc/save_pc to test point_cloud fusion pipeline
    * Modify PointCloud2 topic in rviz to see live point cloud, last saved point cloud, or fused point cloud


<p>**<br>

Running `generate_pc` transforms the PointCloud2 (pc2) into the turtlebot3's frame.<br>

Calling the service `save_pc` publishes the transformed pc2 to `fuse_pc.cpp`.<br>

Therefore `fuse_pc.cpp` only receives one pc2 each time save_pc is called.<br>

Since our project only deals with 5 faces of the object, there is an if statement within the fusion function that will publish the final 5-sided pc2 to `/final_pc` when it counts that 5 pc2 messages have been received. (It then resets the count and clears the output so the user can simply replace the object and continue calling save_pc without having to kill or rerun anything.)<br>

/*Note: this parameter can also be changed from 5 to a ros parameter to allow the user more flexibility in deciding the number of sides they would like to concatenate.*/<br>

Finally, since pointcloud_to_pcd.cpp is run first and set to listen to the `/final_pc` topic, it will only receive the 5-sided (or n-sided if you switch it out with a parameter) pc2, so the user won't have to look through each file in meshlab since there should only be one.<br>

/*Note: the user can also modify other parameters when saving their pcd by editing other parameters in the to pcd node (eg: prefix_, filename_, etc...)*/</p>