cmake_minimum_required(VERSION 2.8.3)
project(ros_base_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  tf
  move_base
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS
  roscpp
  sensor_msgs
  tf
  move_base
)

add_executable(navsPathConverter src/NavsPathConverter.cpp)
target_link_libraries(navsPathConverter ${catkin_LIBRARIES})
add_dependencies(navsPathConverter ${catkin_EXPORTED_TARGETS})
