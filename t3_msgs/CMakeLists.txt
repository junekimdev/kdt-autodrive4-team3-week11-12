cmake_minimum_required(VERSION 3.0.2)
project(t3_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
  message_runtime
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  BoundingBox.msg
  lane_data.msg
  object_data.msg
  stop_line_data.msg
  traffic_light_data.msg
  traffic_light_image.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS message_runtime
)
