cmake_minimum_required(VERSION 3.0.2)
project(mvn_pln)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
  motion_synth
  path_planner
)
catkin_package(
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(mvn_pln_node src/mvn_pln_node.cpp)

add_dependencies(mvn_pln_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(mvn_pln_node
  ${catkin_LIBRARIES}
)

