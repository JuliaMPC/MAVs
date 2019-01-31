#!/bin/bash

# $0 is the script name, $1 is the folder name
# EX: ./sweep.sh "sweepB"

FOLDERNAME="$1"
rm -rf /home/mavs/MAVs/results/$FOLDERNAME/*
rmdir /home/mavs/MAVs/results/$FOLDERNAME
mkdir /home/mavs/MAVs/results/$FOLDERNAME

wait_for_roscore_to_initialize () {
  echo "Entering: wait_for_roscore_to_initialize"
  until rostopic list ; do sleep 1; done
  echo "Exiting: wait_for_roscore_to_initialize"
}
wait_for_roslaunch_pid_to_stop () {
  echo "Entering: wait_for_roslaunch_pid_to_stop"
  while kill -s 0 "$roslaunch_PID"; do
    sleep 2
  done
  echo "Exiting: wait_for_roslaunch_pid_to_stop"
}
kill_roslaunch_pid () {
  echo "Entering: kill_roslaunch_pid"
  kill -INT $roslaunch_PID
  wait_for_roslaunch_pid_to_stop
  sleep 3
  echo "Exiting: kill_roslaunch_pid"
}
kill_roscore_pid () {
  echo "Entering: kill_roscore_pid"
  kill -INT $roscore_PID
  sleep 2
  echo "Exiting: kill_roscore_pid"
}
update_dynamic_ros_parameters () {
  echo "Entering: update_dynamic_ros_parameters"
  for key in "${!parameters[@]}"; do
    rosparam set $key ${parameters[$key]}
  done
  # show where in the loop it is
  echo `rosparam get "/system/nloptcontrol_planner/flags/known_environment"`
  echo `rosparam get "/planner/nloptcontrol_planner/misc/movingObstacles"`
  echo `rosparam get "/case/actual/obstacle/vy"`
  echo `rosparam get "/case/actual/obstacle/radius"`

  # remove so that the script fails if they are not created
  rm /home/mavs/MAVs/ros/src/system/config/case/tmp.yaml
  rm /home/mavs/MAVs/ros/src/system/config/planner/nloptcontrol_planner/tmp.yaml
  # create temp YAML files that can be loaded by the functions: obstacle_avoidance.jl and in the launch file (to programatically create obstacles)
  # while these parameters are currently on the ros server, these functions that need to load these directly from a file
  cd /home/mavs/MAVs/ros/src/system/param_sweep/scripts
  sh ./rosparam_append.sh /home/mavs/MAVs/ros/src/system/config/case/s8Sweep.yaml / /home/mavs/MAVs/ros/src/system/config/case/tmp.yaml
  sh ./rosparam_append.sh /home/mavs/MAVs/ros/src/system/config/planner/nloptcontrol_planner/Dsweep.yaml / /home/mavs/MAVs/ros/src/system/config/planner/nloptcontrol_planner/tmp.yaml
  sleep 2

  echo "Exiting: update_dynamic_ros_parameters"
}
start_roscore () {
  echo "Entering: start_roscore"
  roscore &
  roscore_PID=$!
  wait_for_roscore_to_initialize
  echo "Exiting: start_roscore"
}
run_launchfile () {
  echo "Entering: run_launchfile"
  update_dynamic_ros_parameters
  roslaunch system sweepA.launch &
  roslaunch_PID=$!
  sleep 4
  echo "Exiting: run_launchfile"
}
wait_for_program () {
  echo "Entering: wait_for_program"
  DURATION=500
  START_TIME=$SECONDS
  shutdown=0;
until (($(( SECONDS - START_TIME )) > "$DURATION")) || [[ $shutdown -eq 1 ]]; do
  sleep 1;
  if [[ `rosparam get "system/flags/done"` = "true" ]]; then
    echo "system/flags/done = true"
    shutdown=1;
    sleep 10
  fi
  if [[ `rosparam get "/vehicle_collided"` = "true" ]]; then
    echo "/vehicle_collided = true"
    shutdown=1;
    sleep 10
  fi
done
sleep 5
echo "Exiting: wait_for_program"
}

loop_entry_point () {
  echo "Entering: loop_entry_point"
  cd /home/mavs/MAVs/results
  rosbag record -O tmp.bag /state /nloptcontrol_planner/opt __name:=my_bag &
  run_launchfile
  wait_for_program
  wait 5
  rosnode kill /my_bag
  kill_roslaunch_pid
  kill_roscore_pid
  sleep 20
  echo "Exiting: loop_entry_point"
}

planners=( "true" )
knowns=( "true" "false")
#vys=([-10.,0.,0.,-1.] [-9.5,0.,0.,-1.] [-9.,0.,0.,-1.] [-8.5,0.,0.,-1.] [-8.,0.,0.,-1.] [-7.5,0.,0.,-1.] [-7.,0.,0.,-1.] [-6.5,0.,0.,-1.] [-6.,0.,0.,-1.] [-5.,0.,0.,-1.] [-5.5,0.,0.,-1.] [-5.,0.,0.,-1.] [-4.5,0.,0.,-1.] [-4.,0.,0.,-1.] [-3.5,0.,0.,-1.] [-3.,0.,0.,-1.] [-2.5,0.,0.,-1.] [-2.,0.,0.,-1.] [-1.5,0.,0.,-1.] [-1.,0.,0.,-1.] [-0.5,0.,0.,-1.] [0.,0.,0.,-1.])
#radi=( [0.25,10.,5.,12.] [0.5,10.,5.,12.] [1.,10.,5.,12.] [1.5,10.,5.,12.] [2.,10.,5.,12.] [2.5,10.,5.,12.] [3.,10.,5.,12.] [3.5,10.,5.,12.] [4.,10.,5.,12.] [4.5,10.,5.,12.] [5.,10.,5.,12.] [5.5,10.,5.,12.] [6.5,10.,5.,12.] [7.5,10.,5.,12.] [8.,10.,5.,12.] [8.5,10.,5.,12.] [9.,10.,5.,12.] [9.5,10.,5.,12.] [10.,10.,5.,12.] )
vys=( [-20.,0.,0.,-1.] [-19.,0.,0.,-1.] [-18,0.,0.,-1.] [-17.,0.,0.,-1.] [-16.,0.,0.,-1.] [-15.,0.,0.,-1.] [-14.,0.,0.,-1.] [-13.,0.,0.,-1.] [-12.,0.,0.,-1.] [-11,0.,0.,-1.] [-10.,0.,0.,-1.] [-9,0.,0.,-1.] [-8.,0.,0.,-1.] [-7,0.,0.,-1.] [-6.,0.,0.,-1.] [-5,0.,0.,-1.] [-4.,0.,0.,-1.] [-3,0.,0.,-1.] [-2.,0.,0.,-1.] [-1,0.,0.,-1.] [0.,0.,0.,-1.])
radi=( [9.,10.,5.,12.] [8.,10.,5.,12.] [7.,10.,5.,12.] [6.,10.,5.,12.] [5.,10.,5.,12.]  [4.,10.,5.,12.]  [3.,10.,5.,12.]  [2.,10.,5.,12.]  [1.,10.,5.,12.] [10.,10.,5.,12.])

declare -A parameters
idx=1;
for radius in ${radi[@]}; do
   for vy in ${vys[@]}; do
     for plan in ${planners[@]}; do
       for known in ${knowns[@]}; do
        echo "______________________________________"
        echo "Running for the $idx th time."
        echo "--------------------------------------"
        rm /home/mavs/MAVs/results/tmp.bag
        rosclean purge -y
        parameters["/system/nloptcontrol_planner/flags/known_environment"]=${known}
        parameters["/planner/nloptcontrol_planner/misc/movingObstacles"]=${plan}
        parameters["/case/actual/obstacle/vy"]=${vy}
        parameters["/case/actual/obstacle/radius"]=${radius}
        start_roscore
        sleep 2;
        rosparam set "/case/id" "$FOLDERNAME"test"$idx"
        loop_entry_point
        echo "Entering: postProcess"
        mkdir /home/mavs/MAVs/results/$FOLDERNAME/test"$idx"
        cd /home/mavs/MAVs/results
        #rosbag reindex -f tmp.bag
        rosbag info tmp.bag
        python bag_to_csv.py /home/mavs/MAVs/results/tmp.bag "/home/mavs/MAVs/results/$FOLDERNAME/test"$idx""
        julia plottingData.jl "/home/mavs/MAVs/results/$FOLDERNAME/test"$idx"" "tmp"
        rm /home/mavs/MAVs/results/$FOLDERNAME/test"$idx"/state.csv
        echo "Exiting: postProcess"
        idx=$(( $idx+1 ))
      done
    done
  done
done
