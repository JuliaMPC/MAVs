cmake_minimum_required(VERSION 2.8.3)
project(system)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)
catkin_metapackage()

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS
  roscpp
)

add_executable(system_shutdown src/system_shutdown.cpp)

target_link_libraries(system_shutdown ${catkin_LIBRARIES})

add_dependencies(system_shutdown ${catkin_EXPORTED_TARGETS})
