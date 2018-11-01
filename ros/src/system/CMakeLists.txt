cmake_minimum_required(VERSION 2.8.3)
project(system)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)
catkin_metapackage() # rostest example: catkin_package()

include_directories(
	${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
)

add_executable(system_shutdown src/system_shutdown.cpp)

target_link_libraries(system_shutdown ${catkin_LIBRARIES})

add_dependencies(system_shutdown ${catkin_EXPORTED_TARGETS})


##############
# g Tests
##############
# New added for rostest
if(CATKIN_ENABLE_TESTING)
	message("\n\nGoing to build the tests!\n\n")
	find_package(rostest REQUIRED)
	find_package(gtest)
	# You can add arbitrary new test files in the end of add_rostest_gtest
	add_rostest_gtest(test_mynode test/gtest_launch.launch src/test/mytest.cpp)
	add_rostest(test/demo_test.test) #perhaps

	target_link_libraries(test_mynode ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
endif()
# End rostest
