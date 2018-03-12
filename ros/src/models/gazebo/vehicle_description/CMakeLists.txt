cmake_minimum_required(VERSION 2.8.3)
project(vehicle_description)


find_package(catkin REQUIRED COMPONENTS
  #eigen3
  gazebo_ros
  geometry_msgs
  roscpp
  rospy
  tf
  tf2
  nav_msgs
  sensor_msgs
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gazebo_ros_velodyne_laser
  CATKIN_DEPENDS geometry_msgs roscpp rospy tf tf2 nav_msgs sensor_msgs gazebo_ros
)

add_library(gazebo_ros_velodyne_laser src/GazeboRosVelodyneLaser.cpp)
target_link_libraries(gazebo_ros_velodyne_laser
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  RayPlugin
)

install(TARGETS gazebo_ros_velodyne_laser
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

add_executable(frame_tf_broadcaster src/frame_tf_broadcaster.cpp)
target_link_libraries(frame_tf_broadcaster ${catkin_LIBRARIES})

add_executable(position_broadcaster src/position_broadcaster.cpp)
target_link_libraries(position_broadcaster ${catkin_LIBRARIES})

add_executable(odom src/odom.cpp)
target_link_libraries(odom ${catkin_LIBRARIES})
