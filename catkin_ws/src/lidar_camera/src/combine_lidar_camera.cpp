#include <lidar_camera/pointcloud_to_image.hpp>

int main (int argc, char **argv)
{
  ros::init (argc, argv, "combine_lidar_camera");
  ROS_INFO("Rosnode initialized");

  PointCloudToImage pci; //load up the node

  while(ros::ok())
  {
    ros::spinOnce();
  }
  ROS_INFO("Exiting");
  return 0;
}