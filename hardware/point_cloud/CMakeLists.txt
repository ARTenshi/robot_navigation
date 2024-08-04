cmake_minimum_required(VERSION 3.0.2)
project(point_cloud)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
  pcl_conversions
  pcl_ros
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_service_files(
  FILES
  PointCloudFilter.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)
catkin_package(
  CATKIN_DEPENDS 
  roscpp 
  rospy 
  std_msgs 
  sensor_msgs 
  message_runtime
  pcl_conversions 
  pcl_ros 
  tf2 
  tf2_ros 
  tf2_geometry_msgs 
#  INCLUDE_DIRS include
#  LIBRARIES point_cloud
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
add_executable(passthrough_filter 
src/passthrough.cpp)

add_executable(plane_detector 
  src/plane_detector.cpp)

add_dependencies(plane_detector 
${${PROJECT_NAME}_EXPORTED_TARGETS}
${catkin_EXPORTED_TARGETS})

add_dependencies(passthrough_filter
${${PROJECT_NAME}_EXPORTED_TARGETS} 
${catkin_EXPORTED_TARGETS})

target_link_libraries(plane_detector 
${catkin_LIBRARIES})

target_link_libraries(passthrough_filter 
${catkin_LIBRARIES}
${tf2_LIBRARIES} 
${tf2_ros_LIBRARIES}
${tf2_geometry_msgs_LIBRARIES} )
