cmake_minimum_required(VERSION 2.8.3)
project(visualization)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
)
