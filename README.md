# final-project-blocked
final-project-blocked created by GitHub Classroom

## File structure:
```
--final-project-blocked
  --arm_motion
  --camera_motion
  --camera_reconstruct
    --sudo apt install python3-pcl
```
## Packages & Dependencies

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


    