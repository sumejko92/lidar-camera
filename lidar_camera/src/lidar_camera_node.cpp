#include <lidar_camera/lidar_camera_node.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_camera_node");

  LidarCamera pci;

  ROS_INFO("Node initialized");
  ros::spin;

  return 0;
}