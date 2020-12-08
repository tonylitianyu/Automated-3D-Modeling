# Camera Motion Package
### Author: Blocked

This package contains a tool for aligning the center of the depth camera with the scanning object during initial setup.

### `find_depth` node
The `find_depth` node publishes the depth value of the center <b>n x n</b> pixel. In this way, one can know that if the object is at a the center of the cameraview by checking if the published depth value is at the required distance.


### `camera_motion` python library
This library contains function for calculating the average depth value across space and time to obtain a more stable depth value. For space, it is taking the average depth values of the <b>n x n</b> center pixels. For time, it is taking the average depth values across the past <b>m</b> seconds. This library also comes with unit testing.

#### Usage Instructions:
 1. Rosrun it during the initial hardware setup
 ```
 rosrun camera_motion find_depth
 ```
