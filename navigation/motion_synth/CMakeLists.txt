cmake_minimum_required(VERSION 3.0.2)
project(motion_synth)

find_package(catkin REQUIRED COMPONENTS
  rospy
  actionlib_msgs
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  Joints.msg
  StartAndEndJoints.msg
)

add_action_files(
  FILES
  MotionSynthesis.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
