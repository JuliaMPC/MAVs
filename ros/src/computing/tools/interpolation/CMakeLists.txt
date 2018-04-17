cmake_minimum_required(VERSION 2.8.3)
project(interpolation)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/ap.cpp
  src/integration.cpp
  src/interpolation.cpp
  src/alglibinternal.cpp
  src/linalg.cpp
  src/alglibmisc.cpp
  src/specialfunctions.cpp
  src/solvers.cpp
  src/optimization.cpp
)

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)








