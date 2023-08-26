cmake_minimum_required(VERSION 3.0.2)
project(dataset_cnn)

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
  geometry_msgs
  message_generation
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

# add_library(DataSet
#   include/CNNDataSet/CNNDataSet.h
#   src/CNNDataSet.cpp
# )

# add_dependencies(DataSet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# target_link_libraries(DataSet
#   ${catkin_LIBRARIES}
# )

# add_executable(DataSet_node src/CNNDataSet_node.cpp)

add_executable(gridMap_to_image_server src/gridMap_to_image_server.cpp)

add_executable(traver_train src/TraverDataSet.cpp)

add_executable(index_test src/index_test.cpp)

# add_dependencies(DataSet_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# target_link_libraries(DataSet_node
#   DataSet
#   ${catkin_LIBRARIES}
# )

target_link_libraries(gridMap_to_image_server
  ${catkin_LIBRARIES}
)

target_link_libraries(index_test
  ${catkin_LIBRARIES}
)

target_link_libraries(traver_train
  ${catkin_LIBRARIES}
)


