#include <lidar_camera/lidar_camera.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_camera_node");

  LidarCamera pci;
  pci.init();

  // ROS_INFO_STREAM(pci.nodeName() << " initialized...");

  ROS_INFO("Node initialized");
  ros::spin();

  return 0;
}