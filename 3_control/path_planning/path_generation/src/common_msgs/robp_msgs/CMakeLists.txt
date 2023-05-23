cmake_minimum_required(VERSION 3.0.2)

project(robp_msgs
  VERSION 1.0.0
  DESCRIPTION ""
  LANGUAGES C CXX
)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

add_message_files(
  FILES
  Encoders.msg
  DutyCycles.msg
  flags.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
