cmake_minimum_required(VERSION 2.8.3)
project(contact_sensor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

find_package(gazebo REQUIRED)

catkin_package(
   CATKIN_DEPENDS roscpp std_msgs
)

###########
## Build ##
###########

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIR}
                    ${GAZEBO_INCLUDE_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

## Declare a C++ library
add_library(${PROJECT_NAME}_mavs SHARED src/ContactPlugin.cpp)
target_link_libraries(${PROJECT_NAME}_mavs ${catkin_LIBRARIES} ${GAZEBO_libraries})
