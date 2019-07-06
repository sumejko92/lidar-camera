cmake_minimum_required(VERSION 2.8.3)
project(lidar_camera)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  image_geometry
  image_transport
  cv_bridge
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/LidarCam.cfg
)

catkin_package(
  INCLUDE_DIRS include
 # LIBRARIES 
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros image_geometry image_transport cv_bridge dynamic_reconfigure
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
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)