cmake_minimum_required(VERSION 2.8.3)
project(lidar_camera)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
  image_geometry
)

catkin_package(
  INCLUDE_DIRS include
 # LIBRARIES 
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs pcl_ros image_geometry
 # DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  include/${PROJECT_NAME}/${PROJECT_NAME}_node.h
  src/${PROJECT_NAME}_node.cpp
)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

add_executable(combine_lidar_camera src/combine_lidar_camera.cpp)
target_link_libraries(combine_lidar_camera pointcloud_to_image_lib ${OpenCV_LIBS} ${catkin_LIBRARIES})
add_dependencies(combine_lidar_camera lidar_camera_generate_messages_cpp ${catkin_EXPORTED_TARGETS})