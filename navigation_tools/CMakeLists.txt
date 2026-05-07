cmake_minimum_required(VERSION 3.0.2)
project(navigation_tools)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################


## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  geometry_msgs
#)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
