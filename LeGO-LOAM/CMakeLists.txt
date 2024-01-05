cmake_minimum_required(VERSION 2.8.3)
project(lego_loam_bor)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3 -g ")

#

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  image_transport
  rosbag_storage

  pcl_ros
  pcl_conversions

  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  cloud_msgs
)

find_package(GTSAM REQUIRED QUIET)
find_package(PCL REQUIRED QUIET)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cloud_msgs rosbag_storage
  DEPENDS PCL
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
)

link_directories(
	include
	${PCL_LIBRARY_DIRS}
	${GTSAM_LIBRARY_DIRS}
)

add_library(lego_loam_bor SHARED src/imageProjection.cpp
                                 src/featureAssociation.cpp
                                 src/mapOptimization.cpp
                                 src/transformFusion.cpp)

add_executable(lego_loam_bag src/main_bag.cpp)
add_dependencies(lego_loam_bag ${catkin_EXPORTED_TARGETS} cloud_msgs_gencpp lego_loam_bor)
target_link_libraries(lego_loam_bag ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtsam lego_loam_bor)

add_executable(br_lego_loam_bor src/brmain.cpp)
add_dependencies(br_lego_loam_bor ${catkin_EXPORTED_TARGETS} cloud_msgs_gencpp lego_loam_bor)
target_link_libraries(br_lego_loam_bor ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtsam lego_loam_bor)

