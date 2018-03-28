cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(set_step_size)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
)

find_package(gazebo REQUIRED)

catkin_package(
)

include_directories(${GAZEBO_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(set_step_size SHARED set_step_size.cpp)
target_link_libraries(set_step_size
${GAZEBO_LIBRARIES}
${catkin_LIBRARIES}
)
