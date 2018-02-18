#include <lidar_camera/pointcloud_to_image.hpp>

// Declare a test
TEST(PointcloudToImageTest, valuesTest)
	{
		ros::Time::init();

		EXPECT_EQ(5, 5);

		// in order to test the functions from the class fake publishers need to be created

		// ros::NodeHandle nh_;
		// ros::Publisher fake_cloud_pub;
		// sensor_msgs::PointCloud2 fake_cloud;
		// fake_cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("/lidar0/velodyne_points", 1);
		//...
	}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
