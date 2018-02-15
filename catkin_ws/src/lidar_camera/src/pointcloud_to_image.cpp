/**
 * @file pointcloud_to_image.cpp
 *
 * @brief PointCloudToImage class that combines 3D LIDAR and usb camera data
 * @author Pavel Shumejko,   <p_sumejko@yahoo.com>
 *
 * Created on: Feb 13, 2018
 */

#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/transforms.h>

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include <string>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>


/**
 * This node subscribes to a 3D LIDAR and an usb camera topics and:
 * 1. Publishes a filtered PointCloud2, that includes all the LIDAR points that overlap the camera image with respective RGB of the point in the enviroment 
 * 2. Depth image that is of the same size and FOV as the camera image and has metric depth data for each of the pixels
 */

class PointCloudToImage
{
public:
  void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud)
  {
    PointCloud cloud_xyz;
    sensor_msgs::PointCloud2 transformed_cloud;
    sensor_msgs::PointCloud2 filtered_cloud;

    pcl::PointCloud<pcl::PointXYZRGB> cloud_xyzrgb;
    pcl::PointCloud<pcl::PointXYZRGB> filtered_cloud_xyzrgb;

    if ((cloud->width * cloud->height) == 0)
    {
      ROS_WARN("CLOUD NOT DENSE");
      return;
    }

    tf::StampedTransform transform;

    try
    {
      listener.waitForTransform("cam0", "lidar0", ros::Time(0), ros::Duration(10.0));
      listener.lookupTransform("cam0", "lidar0", ros::Time(0), transform);
    }
    catch (tf::TransformException ex) 
    {
      ROS_ERROR("Transform exception %s",ex.what());
    }

    pcl_ros::transformPointCloud("cam0", transform, *cloud, transformed_cloud);

    try
    {
      pcl::fromROSMsg (transformed_cloud, cloud_xyz); //convert to pcl-xyz cloud
    }
    catch (std::runtime_error e)
    {
      ROS_ERROR_STREAM("Error when converting from cloud message: "<< e.what());
      return;
    }

    pcl::copyPointCloud(cloud_xyz, cloud_xyzrgb);

    cv::Point2d uv_pixel;

    for(const pcl::PointXYZRGB &point : cloud_xyzrgb.points)
    {
      if(point.z > 0.0) 
      {
        cv::Point3d pt_cv(point.x, point.y, point.z);
        uv_pixel = cam_model_.project3dToPixel(pt_cv);
        if(uv_pixel.x > 0 and uv_pixel.x < 1280 and uv_pixel.y > 0 and uv_pixel.y < 720) 
          {
            cv::Vec3b colour = frame.at<cv::Vec3b>(cv::Point(uv_pixel.x, uv_pixel.y));
            pcl::PointXYZRGB temp = point;
            temp.r = colour.val[2];
            temp.g = colour.val[1];
            temp.b = colour.val[0];
            filtered_cloud_xyzrgb.push_back(temp);
          }
      }
    }

    pcl::toROSMsg (filtered_cloud_xyzrgb, filtered_cloud);

    filtered_cloud.header.frame_id = "cam0";

    filtered_pub.publish(filtered_cloud);
  }

  void image_cb (const sensor_msgs::ImageConstPtr& original_image)
  {
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(original_image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    frame = cv_ptr->image;

    //output_image_pub.publish(cv_ptr->toImageMsg());
  }

  void camera_info_cb(const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    cam_model_.fromCameraInfo(info_msg);
  }

  // Constructor
  PointCloudToImage () : cloud_topic_("/lidar0/velodyne_points"), image_topic_("/cam0/image_rect_color"), 
  filtered_cloud_topic_("/lidar0/filtered_points"), image_projected_topic_("/output_image"), it_(nh_)
  {
    cloud_sub_ = nh_.subscribe (cloud_topic_, 30, &PointCloudToImage::cloud_cb, this);
    image_sub_ = it_.subscribe (image_topic_, 1, &PointCloudToImage::image_cb, this);
    camera_info_sub_ = nh_.subscribe("/cam0/camera_info", 1, &PointCloudToImage::camera_info_cb, this);

    filtered_pub = nh_.advertise<sensor_msgs::PointCloud2>(filtered_cloud_topic_, 1);
    output_image_pub = it_.advertise(image_projected_topic_, 1);

    //print some info about the node
    std::string r_ct = nh_.resolveName (cloud_topic_);
    std::string r_it = nh_.resolveName (image_topic_);
    std::string r_ot = nh_.resolveName (image_projected_topic_);
    ROS_INFO_STREAM("Listening for incoming pcl data on topic " << r_ct );
    ROS_INFO_STREAM("Listening for incoming images on topic " << r_it );
    ROS_INFO_STREAM("Publishing output image on topic " << r_ot );
  }
private:
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
  
  std::string cloud_topic_; //default point cloud input
  std::string image_topic_; //default image input
  std::string filtered_cloud_topic_; //default pointcloud output
  std::string image_projected_topic_; //default output image

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

int main (int argc, char **argv)
{
  ros::init (argc, argv, "pointcloud_to_image");
  ROS_INFO("Rosnode initialized");

  PointCloudToImage pci; //load up the node

  while(ros::ok())
  {
    ros::spinOnce();
  }
  return 0;
}