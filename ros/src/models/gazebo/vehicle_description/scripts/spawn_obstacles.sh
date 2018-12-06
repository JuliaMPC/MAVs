#!/bin/bash

case_params_path=`rosparam get /case_params_path`
num_of_obstacles=`rosparam get /case/actual/obstacle/num`
for ((idx=0; idx<$num_of_obstacles; idx++))
do
roslaunch vehicle_description obstacles.launch case_params_path:=$case_params_path obstacle_idx:=$idx
done
rosparam set system/vehicle_description/flags/obstacles_spawned true
