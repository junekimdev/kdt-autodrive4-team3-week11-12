cmake_minimum_required(VERSION 3.0.2)
project(t3_sensor_lane_cam)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  t3_msgs
)
#find_package(OpenCV REQUIRED)
find_package(OpenCV 4.5.5 REQUIRED PATHS ~/OpenCV4/install/lib/cmake/opencv4)

catkin_package(
 INCLUDE_DIRS include
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
