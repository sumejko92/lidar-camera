# Lidar-camera

This package takes inputs from a 3D lidar and separate camera, combines the inputs in such a way as to produce two separate outputs:
1. A PointCloud2, that includes all the LIDAR points that overlap the camera image with added R, G and B data channels that contain the color of the point in the environment.
2. A standard uint16 depth image that is of the same size and FOV as the camera image and has metric depth data for each of the pixels.

## Setup
ROS distribution: Melodic
Ubuntu: Ubuntu 18.04

Clone the package in your catkin workspace and build to resolve missing dependencies.

## How to run
Before running the roslaunch command make sure that you have sensor_msgs/PointCloud2 sensor_msgs/Image topics up as well as their respective tfs.
No filtered pointcloud and depth image messages will not be published untill there are subscribers to their topics.   

Run the following command to launch the nodes:
```
roslaunch lidar_camera test_lidar_camera.launch
```
This will start the lidar_camera_node and rviz with respective config displaying the filtered colored pointclound and depth image.

## Results

Filtered and colored pointcloud:

<p align="center">
  <img src="https://raw.githubusercontent.com/sumejko92/lidar-camera/master/catkin_ws/src/results/rviz.png" width="450" /> 
</p>

Depth image:

<p align="center">
  <img src="https://raw.githubusercontent.com/sumejko92/lidar-camera/master/catkin_ws/src/results/depth.png" width="450" /> 
</p>
