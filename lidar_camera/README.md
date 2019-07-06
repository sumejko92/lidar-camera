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

Dynamic reconfigure can be used to apply filtering and tinker with the depth image params.

## Results

Filtered and colored pointcloud:

<p align="center">
  <img src="https://raw.githubusercontent.com/sumejko92/lidar-camera/melodic-devel/lidar_camera/results/rviz.png" width="450" /> 
</p>

Depth image:

<p align="center">
  <img src="https://raw.githubusercontent.com/sumejko92/lidar-camera/melodic-devel/lidar_camera/results/depth.png" width="450" /> 
</p>
