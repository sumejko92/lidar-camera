#include <lidar_camera/pointcloud_to_image.hpp>

  void PointCloudToImage::cloudCb (const sensor_msgs::PointCloud2ConstPtr& cloud)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
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

    float distance;
    std::vector<uint16_t> vec;
    cv::Mat depthMat = cv::Mat::zeros(this->frame.size(), CV_16UC1);
    //cv::Mat depthf = cv::Mat::zeros(this->frame.size(), CV_8UC1);

    try
    {
      this->listener.waitForTransform("cam0", "lidar0", ros::Time(0), ros::Duration(10.0));
      this->listener.lookupTransform("cam0", "lidar0", ros::Time(0), transform);
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
            cv::Vec3b colour = this->frame.at<cv::Vec3b>(cv::Point(uv_pixel.x, uv_pixel.y));
            pcl::PointXYZRGB temp = point;
            temp.r = colour.val[2];
            temp.g = colour.val[1];
            temp.b = colour.val[0];
            filtered_cloud_xyzrgb.push_back(temp);

            distance = getDistanceToPoint(pt_cv);
     				
     				/// not scaled
     				depthMat.at<uint16_t>(cv::Point(uv_pixel.x, uv_pixel.y)) = static_cast<uint16_t>(distance * 1000);
     				//vec.push_back(distance * 1000);

     				/// scaled (to use comment the "not scaled" lines above)
     				// depthMat.at<uint16_t>(cv::Point(uv_pixel.x, uv_pixel.y)) = static_cast<uint16_t>(mapValue(distance, 0, 10, 0, 65535));
     				// vec.push_back(static_cast<uint16_t>(mapValue(distance, 0, 10, 0, 65535)));
          }
      }
    }
    cv::Mat temp, result;
    dilate(depthMat, result, cv::Mat(), cv::Point(-1, -1), 10, 1, 1);

    //depthMat.convertTo(depthf, CV_8UC1, 255.0/getMaxMin(vec).second);
		//dilate(depthf, temp, cv::Mat(), cv::Point(-1, -1), 10, 1, 1);
		//temp.convertTo(result, CV_16UC1, getMaxMin(vec).second/255.0);

    cv_bridge::CvImage out_msg;
		out_msg.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
		out_msg.image = result;
		this->output_image_pub.publish(out_msg.toImageMsg());

    pcl::toROSMsg (filtered_cloud_xyzrgb, filtered_cloud);
    filtered_cloud.header.frame_id = "cam0";
    this->filtered_pub.publish(filtered_cloud);
  }

  void PointCloudToImage::imageCb(const sensor_msgs::ImageConstPtr& original_image)
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

    this->frame = cv_ptr->image;
  }

  void PointCloudToImage::cameraInfoCb(const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    this->cam_model_.fromCameraInfo(info_msg);
  }

  float PointCloudToImage::getDistanceToPoint(cv::Point3d point)
  {
  	return sqrt(pow((point.x), 2) + pow((point.y), 2) + pow((point.z), 2));
  }

  float PointCloudToImage::mapValue(uint16_t s, uint16_t a1, uint16_t a2, uint16_t b1, uint16_t b2)
	{
    return b1 + (s - a1) * (b2 - b1) / (a2 - a1);
	}

	std::pair<uint16_t, uint16_t> PointCloudToImage::getMaxMin(std::vector<uint16_t> vec)
  {
  	uint16_t max_val = 0;
  	uint16_t min_val = 0;
  	std::pair <uint16_t, uint16_t> result;

  	for(uint16_t val : vec)
  	{
  		if(val > max_val)
  		{
  			max_val = val;
  		}
  		if(val < min_val)
  		{
  			min_val = val;
  		}
  	}
  	result = std::make_pair(min_val, max_val);
  	return result;
  }