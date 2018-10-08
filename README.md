# Lidar-camera

This package takes inputs from a 3D lidar and separate camera, combines the inputs in such a way as to produce two separate outputs:
1. A PointCloud2, that includes all the LIDAR points that overlap the camera image with added R, G and B data channels that contain the color of the point in the environment.
2. A standard uint16 depth image that is of the same size and FOV as the camera image and has metric depth data for each of the pixels. 

## Setup
ROS distribution: Jade
Ubuntu: Ubuntu 14.04

Since there were conflicts between the opencv versions(ROS Jade has dependencies to opencv2.4) [cv_bridge](https://github.com/ros-perception/vision_opencv) and [image_pipeline](https://github.com/ros-perception/image_pipeline) source code was cloned into the workspace to be recompiled. 

## How to run
Before running the roslaunch command make sure that you have created a "bag_files" folder in the lidar_camera package and a rosbag "bag1.bag" file is inside.  
Run the following command to launch the nodes:
```
roslaunch lidar_camera combine_lidar_camera.launch
```
This will start the rosbag, combine_lidar_camera node and image_view for displaying the depth image.

## Results

Filtered and colored pointcloud:

<p align="center">
  <img src="https://raw.githubusercontent.com/Dzvezdana/lidar-camera/tree/master/catkin_ws/src/results/rviz.png" width="300" /> 
</p>

Depth image:

<p align="center">
  <img src="https://raw.githubusercontent.com/Dzvezdana/lidar-camera/tree/master/catkin_ws/src/results/depth.png width="300" /> 
</p>