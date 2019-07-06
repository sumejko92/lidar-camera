#include <lidar_camera/lidar_camera.h>

void LidarCamera::init() {
  cloud_in_sub_ =
      nh_.subscribe(cloud_in_topic_, queue_size_, &LidarCamera::cloudCb, this);
  image_in_sub_ =
      it_.subscribe(image_in_topic_, queue_size_, &LidarCamera::imageCb, this);

  processed_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
      processed_cloud_topic_, queue_size_);
  depth_image_pub = it_.advertise(depth_out_topic_, queue_size_);

  dynamic_reconfigure::Server<DynamicConfig>::CallbackType cb =
      boost::bind(&LidarCamera::reconfigureCb, this, _1, _2);
  reconfig_server_.setCallback(cb);

  // Debug info
  ROS_INFO_STREAM("Listening for incoming pointcloud data on topic "
                  << nh_.resolveName(cloud_in_topic_));
  ROS_INFO_STREAM("Listening for incoming images on topic "
                  << nh_.resolveName(image_in_topic_));
  ROS_INFO_STREAM("Publishing filtered point cloud on topic "
                  << nh_.resolveName(processed_cloud_topic_));
  ROS_INFO_STREAM("Publishing output image on topic "
                  << nh_.resolveName(depth_out_topic_));

  // wait for camera info
  while (ros::ok()) // TODO: loop needs better handling
  {
    auto info_msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(
        camera_info_topic_, ros::Duration(5));

    if (info_msg) {
      cam_model_.fromCameraInfo(info_msg);
      break;
    }
    ROS_WARN("No camera info received");
  }
}

std::string LidarCamera::nodeName() { return ros::this_node::getName(); }

void LidarCamera::reconfigureCb(DynamicConfig &config, uint32_t level) {
  range_min_ = config.range_min;
  range_max_ = config.range_max;
  fill_depth_image_ = config.fill_depth_image;
  kernel_size_ = config.kernel_size;
  dilation_itr_ = config.dilation_itr;
}

void LidarCamera::cloudCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
  // check if there are any subscribers already for this topics
  bool process_cloud = processed_cloud_pub_.getNumSubscribers() > 0;
  bool process_depth = depth_image_pub.getNumSubscribers() > 0;

  // Avoid poitcloud processing if none is subscribed
  if (!process_cloud)
    return;

  // return if the poitcloud is empty
  if (cloud_msg->data.empty()) {
    ROS_WARN("Received empty pointcloud");
    return;
  }

  // return if there is no rgb image
  if (!cv_ptr_) {
    ROS_WARN("No image received");
    return;
  }

  // check if cloud is dense
  if ((cloud_msg->width * cloud_msg->height) == 0) {
    ROS_WARN("Cloud not dense");
    return;
  }

  tf::StampedTransform transform;
  // check if the tranform exists
  try {
    listener_.waitForTransform(cam_model_.tfFrame(), cloud_msg->header.frame_id,
                               ros::Time(0), ros::Duration(10.0));
    listener_.lookupTransform(cam_model_.tfFrame(), cloud_msg->header.frame_id,
                              ros::Time(0), transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("Transform exception %s", ex.what());
    return;
  }

  sensor_msgs::PointCloud2 transformed_cloud_msg;
  pcl_ros::transformPointCloud(cam_model_.tfFrame(), transform, *cloud_msg,
                               transformed_cloud_msg);

  pcl::PointCloud<pcl::PointXYZ> cloud_XYZ;

  try {
    pcl::fromROSMsg(transformed_cloud_msg,
                    cloud_XYZ); // convert to pcl-xyz cloud
  } catch (std::runtime_error e) {
    ROS_ERROR_STREAM("Error when converting from cloud message: " << e.what());
    return;
  }

  pcl::PointCloud<pcl::PointXYZRGB> processed_cloud_XYZRGB;
  cv::Mat depthMat(cv_ptr_->image.size(), CV_16UC1, 0.0);

  for (const auto &point : cloud_XYZ) {
    // apply range filtering
    if (inRange(point)) {
      auto uv_pixel = cam_model_.project3dToPixel({point.x, point.y, point.z});
      if (inFOV(uv_pixel, cv_ptr_->image)) {
        auto rgb_colour = cv_ptr_->image.at<cv::Vec3b>(uv_pixel.x, uv_pixel.y);
        processed_cloud_XYZRGB.push_back(createPointXYZRGB(
            point, rgb_colour[0], rgb_colour[1], rgb_colour[2]));

        depthMat.at<uint16_t>(cv::Point(uv_pixel.x, uv_pixel.y)) =
            static_cast<uint16_t>(point.z * std::milli::den);
      }
    }
  }

  // convert the pcl pointcloud to a ROS sensor_msgs::PointCloud2 msg and
  // publish it
  sensor_msgs::PointCloud2 processed_cloud_msg;
  pcl::toROSMsg(processed_cloud_XYZRGB, processed_cloud_msg);
  processed_cloud_msg.header.frame_id = cam_model_.tfFrame();
  processed_cloud_pub_.publish(processed_cloud_msg);

  // do not do any processing if there are no depth image subscribers to the
  // given topics
  if (!process_depth)
    return;

  handleDepthImage(depthMat);
}

void LidarCamera::imageCb(const sensor_msgs::ImageConstPtr &image_msg) {
  try {
    cv_ptr_ = cv_bridge::toCvShare(image_msg);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}

void LidarCamera::handleDepthImage(cv::Mat &depth_img) {

  dilate(depth_img, depth_img,
         cv::Mat::ones(kernel_size_, kernel_size_, CV_32F), cv::Point(-1, -1),
         dilation_itr_, 1, 1);

  if (fill_depth_image_)
    fillDepthImage(depth_img);

  cv_bridge::CvImage out_msg;
  out_msg.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  out_msg.image = depth_img;
  depth_image_pub.publish(out_msg.toImageMsg());
}

void LidarCamera::fillDepthImage(cv::Mat &depth_image) {
  int fill;
  bool colulm_toggle;
  for (int i = 0; i < depth_image.cols; i++) {
    fill = 0;
    colulm_toggle = false;
    for (int j = 0; j < depth_image.rows; j++) {
      if (fill != 0 and depth_image.at<uint16_t>(cv::Point(i, j)) == 0) {
        if (!colulm_toggle) {
          for (int k = 0; k < j; k++) {
            depth_image.at<uint16_t>(cv::Point(i, k)) = fill;
          }
          colulm_toggle = true;
        }
        depth_image.at<uint16_t>(cv::Point(i, j)) = fill;
      }

      if (depth_image.at<uint16_t>(cv::Point(i, j)) != 0) {
        fill = depth_image.at<uint16_t>(cv::Point(i, j));
      }
    }
  }
}

pcl::PointXYZRGB LidarCamera::createPointXYZRGB(const pcl::PointXYZ &pt,
                                                const uint8_t r,
                                                const uint8_t g,
                                                const uint8_t b) {

  pcl::PointXYZRGB point;

  point.x = pt.x;
  point.y = pt.y;
  point.z = pt.z;
  point.data[3] = 1.0f;
  point.r = r;
  point.g = g;
  point.b = b;
  point.a = 255;

  return point;
}