cmake_minimum_required(VERSION 2.8.3)
project(lidar_camera)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  image_geometry
  image_transport
  cv_bridge
)

catkin_package(
  INCLUDE_DIRS include
 # LIBRARIES 
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros image_geometry image_transport cv_bridge
 # DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  include/${PROJECT_NAME}/${PROJECT_NAME}.h
  src/${PROJECT_NAME}_node.cpp
  src/${PROJECT_NAME}.cpp
)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)