cmake_minimum_required(VERSION 2.8.3)
project(visualization)

find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(catkin REQUIRED COMPONENTS rospy)
find_package(catkin REQUIRED COMPONENTS ros_chrono_msgs)
find_package(catkin REQUIRED COMPONENTS nloptcontrol_planner)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
  CATKIN_DEPENDS rospy
  CATKIN_DEPENDS ros_chrono_msgs
  CATKIN_DEPENDS nloptcontrol_planner
)

install(DIRECTORY visualization
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
