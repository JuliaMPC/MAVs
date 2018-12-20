#!/bin/bash

initialized=$(rosparam get system/flags/initialized)
while [ "$initialized" = false ]
do
initialized=$(rosparam get system/flags/initialized)
done
roslaunch visualization main_py.launch
