cmake_minimum_required(VERSION 3.0.2)
project(cnn_dataset)

set(
  CATKIN_PACKAGE_DEPENDENCIES 
    eigen_conversions
    grid_map_core
    grid_map_ros
    grid_map_msgs
    grid_map_demos
    kindr
    kindr_ros
    message_filters
    pcl_ros
    roscpp
    sensor_msgs
    std_msgs
    tf
    tf_conversions
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
  roscpp
  rospy
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
    ${Eigen_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}_library
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES} message_runtime
  DEPENDS
    Boost
)


include_directories(
  include
  SYSTEM
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
)

###########
# Library #
###########
add_executable(data_set src/cnn_dataset_node.cpp)


target_link_libraries(data_set
  ${catkin_LIBRARIES}
)
