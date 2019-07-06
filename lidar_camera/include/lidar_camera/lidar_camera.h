/**
 * @file lidar_camera.h
 *
 * @brief LidarCamera class that contains fuctions for combining 3D LIDAR and
 * usb camera data
 * @author Pavel Shumejko,   <p_sumejko@yahoo.com>
 *
 * Created on: July 4, 2019
 */

#pragma once

#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>

#include <ratio>
#include <string>

/**
 * Class that contains subscribers for LIDAR [sensor_msgs/PointCloud2] and usb
 * camera topic [sensor_msgs/Image] and:
 * 1. Publishes a filtered PointCloud2, that includes all the LIDAR points that
 * overlap the camera image with respective RGB of the point in the enviroment
 * 2. Depth image that is of the same size and FOV as the camera input image and
 * has metric depth data(in mm) for each of the pixels
 */

class LidarCamera {
public:
  // Constructor
  LidarCamera() : it_(nh_){};

  /**
   * @brief default initialization
   */
  void init();

  /**
   * @brief returns node name
   */
  std::string nodeName();

  // TODO
  // dynamic reconfigure

private:
  // Constants
  static constexpr const char *cloud_in_topic_ = "cloud_in"; // pointcloud input
  static constexpr const char *image_in_topic_ = "image_in"; // raw image input
  static constexpr const char *camera_info_topic_ =
      "camera_info"; // camera info
  static constexpr const char *processed_cloud_topic_ =
      "processed_points"; // processed pointcloud output
  static constexpr const char *depth_out_topic_ =
      "depth_out"; // depth image output

  static constexpr const int queue_size_ = 1;

  // Subscribers
  ros::Subscriber cloud_in_sub_;             // pointcloud subscriber
  image_transport::Subscriber image_in_sub_; // image subscriber
  ros::Subscriber camera_info_sub_;          // cameraInfo subscriber

  // Publishers
  ros::Publisher processed_cloud_pub_; // processed RGB pointcloud publisher
  image_transport::Publisher depth_image_pub; // output depth image publisher

  // Parameters
  double range_min_ = 0.0;
  double range_max_ = 30.0;
  int kernel_size_ = 3;
  int dilation_itr_ = 10;

  // Variables
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_geometry::PinholeCameraModel cam_model_;
  tf::TransformListener listener_;
  cv_bridge::CvImageConstPtr cv_ptr_;

  // Functions

  /**
   * @brief handles pointcloud callback, processed cloud and invokes
   * handleDepthImage()
   * @param cloud_msg -> sensor_msgs::PointCloud2 msg
   */
  void cloudCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);

  /**
   * @brief image callback
   * @param image_msg -> sensor_msgs::Image msg
   */
  void imageCb(const sensor_msgs::ImageConstPtr &image_msg);

  /**
   * @brief creates and publishes a depth image based on pointcloud data
   * @param img_in -> cv::Mat input not filled image
   */
  void handleDepthImage(cv::Mat &img_in);

  /**
   * @brief fills the blank spots of the depth image with first available color
   * columnvise
   * @param depth_image -> cv::Mat input depth image to be filled
   * @TODO: Looks nasty, needs to be improved
   */
  void fillDepthImage(cv::Mat &depth_image);

  /**
   * @brief checks if pixel is in FOV of an given image
   * @param uv -> cv::Point2i pixel
   * @param img -> cv::Mat image
   * @return returns true if pixel is in FOV of image
   */
  inline bool inFOV(const cv::Point2i &uv, const cv::Mat &img) {
    return uv.x > 0 and uv.x < img.cols and uv.y > 0 and uv.y < img.rows;
  };

  /**
   * @brief checks if a pointcloud point is in desired range
   * @param uv -> cpcl::PointXYZ point
   * @return returns true if pointcloud point is in desired range
   */
  inline bool inRange(const pcl::PointXYZ &pt) {
    return pt.z >= range_min_ && pt.z <= range_max_;
  };

  /**
   * @brief helper function to construcst XYZRGB point from XYZ and r, g, b
   * values
   * @param pt -> pcl::PointXYZ point
   * @param r, g, b -> uint8_t color values
   * @return returns pcl::PointXYZRGB point
   */
  pcl::PointXYZRGB createPointXYZRGB(const pcl::PointXYZ &pt, const uint8_t r,
                                     const uint8_t g, const uint8_t b);
};