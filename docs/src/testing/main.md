## To test (Currently not integrated with .travis)
Following code will run the test on ros node and library to check if everything is correct. For library gtest, it's always true since specific test on each library hasn't been finished. Then for the ros node test, it will test demoA.launch as example.
```
$ cd MAVs/ros
$ catkin_make run_tests
```
