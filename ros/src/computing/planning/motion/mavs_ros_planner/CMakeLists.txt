cmake_minimum_required(VERSION 2.8.3)
project(ros_base_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf
  move_base
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  sensor_msgs
  tf
  move_base
)
