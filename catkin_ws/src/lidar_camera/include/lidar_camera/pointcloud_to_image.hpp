/**
 * @file pointcloud_to_image.hpp
 *
 * @brief PointCloudToImage class that contains fuctions for combining 3D LIDAR and usb camera data
 * @author Pavel Shumejko,   <p_sumejko@yahoo.com>
 *
 * Created on: Feb 18, 2018
 */

#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/photo/photo.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/transforms.h>

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <string>
#include <utility>
#include <gtest/gtest.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>

/**
 * Class that subscribes to a 3D LIDAR and an usb camera topics and:
 * 1. Publishes a filtered PointCloud2, that includes all the LIDAR points that overlap the camera image with respective RGB of the point in the enviroment 
 * 2. Depth image that is of the same size and FOV as the camera image and has metric depth data(in mm) for each of the pixels
 */

class PointCloudToImage
{
public:

  /**
   * @brief velodyne_points callback
   * @param cloud sensor_msgs::PointCloud2 cloud published on /lidar0/velodyne_points topics
   */
  void cloudCb(const sensor_msgs::PointCloud2ConstPtr& cloud);
  
  /**
   * @brief image_rect_color callback
   * @param original_image sensor_msgs::Image published on /cam0/image_rect_color
   */
  void imageCb(const sensor_msgs::ImageConstPtr& original_image);

  /**
   * @brief camera_info callback
   * @param info_msg sensor_msgs::CameraInfo published on /cam0/camera_info
   */
  void cameraInfoCb(const sensor_msgs::CameraInfoConstPtr& info_msg);

  /**
   * @brief calculates the distance to a 3D point
   * @param point cv::Point3d in 3D space(x, y, z)
   * @return returns the distance from the origin to the point
   */
  float getDistanceToPoint(cv::Point3d point);

  /**
   * @brief fills the blank spots of the depth image with first available color columnvise
   * @param depth_image input not filled image
   * @return returns a filled depth image
   */
  cv::Mat fillDepthImage(cv::Mat depth_image);

   /*
    * Construct a PointCloudToImage object which subscribes to the point cloud nad image data from the sensors. 
    * Also creates publishers for the filtered rgb cloud and depth image 
    */
  PointCloudToImage() : cloud_topic_("/lidar0/velodyne_points"), image_topic_("/cam0/image_rect_color"), 
  filtered_cloud_topic_("/lidar0/filtered_points"), depth_image_topic_("/output_image"), it_(nh_)
  {
    filtered_pub = nh_.advertise<sensor_msgs::PointCloud2>(filtered_cloud_topic_, 1);
    output_image_pub = it_.advertise(depth_image_topic_, 1);

    camera_info_sub_ = nh_.subscribe("/cam0/camera_info", 1, &PointCloudToImage::cameraInfoCb, this);
    image_sub_ = it_.subscribe(image_topic_, 1, &PointCloudToImage::imageCb, this);
    cloud_sub_ = nh_.subscribe(cloud_topic_, 30, &PointCloudToImage::cloudCb, this);

    //print some info about the node
    std::string r_ct = nh_.resolveName(cloud_topic_);
    std::string r_it = nh_.resolveName(image_topic_);
    std::string r_ft = nh_.resolveName(filtered_cloud_topic_);
    std::string r_ot = nh_.resolveName(depth_image_topic_);
    ROS_INFO_STREAM("Listening for incoming point cloud data on topic " << r_ct );
    ROS_INFO_STREAM("Listening for incoming images on topic " << r_it );
    ROS_INFO_STREAM("Publishing filtered point cloud on topic " << r_ft );
    ROS_INFO_STREAM("Publishing output image on topic " << r_ot );
  }
private:
  std::string cloud_topic_; //default point cloud input
  std::string image_topic_; //default image input
  std::string filtered_cloud_topic_; //default pointcloud output
  std::string depth_image_topic_; //default output image

  ros::NodeHandle nh_;
  ros::Subscriber cloud_sub_; // pointcloud subscriber
  ros::Subscriber camera_info_sub_; // cameraInfo subscriber
  ros::Publisher filtered_pub; // filtered cloud publisher

  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_; //image subscriber
  image_transport::Publisher output_image_pub; // output image publisher

  image_geometry::PinholeCameraModel cam_model_;

  tf::TransformListener listener;

  cv::Mat frame;
};