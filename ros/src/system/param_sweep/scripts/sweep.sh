#!/bin/bash

# $0 is the script name, $1 is the folder name
# EX: ./sweep.sh "sweepB"

FOLDERNAME="$1"
rm -rf /home/mavs/MAVs/results/$FOLDERNAME/* &>/dev/null
rmdir /home/mavs/MAVs/results/$FOLDERNAME &>/dev/null
mkdir /home/mavs/MAVs/results/$FOLDERNAME &>/dev/null

DEBUG="true"

convertsecs() {
 ((h=${1}/3600))
 ((m=(${1}%3600)/60))
 ((s=${1}%60))
 printf "%02d:%02d:%02d\n" $h $m $s
}

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
  sleep 2
  rosnode kill /my_bag
  if [[ `rosparam get "/system/flags/goal_attained"` = "true" ]]; then
    if [[ `rosparam get "system/nloptcontrol_planner/flags/known_environment"` = "true" ]]; then
      known_RESULTS=$(( $known_RESULTS + 1))
    else
      unknown_RESULTS=$(( $unknown_RESULTS + 1))
    fi
  fi
  cd /home/mavs/MAVs/results/$FOLDERNAME/test"$idx"
  rosparam dump dump.yaml
  kill_roslaunch_pid
  kill_roscore_pid
  sleep 20
  echo "Exiting: loop_entry_point"
}

planners=( "true" )
knowns=( "true" "false")
known_RESULTS=0;
unknown_RESULTS=0;
NUMTESTS=200;
num=$(( $NUMTESTS*2 )); # 2 = number of combinations of planners and knowns

# generate dat
rl=1; # cannot be a float
ru=10; # cannot be a float
declare -a radi
for ((i=0;i<NUMTESTS;i+=1));
do
  radi[${i}]=[$((rl+RANDOM%(ru-rl))).$((RANDOM%99)),10.,5.,12.]
done
#echo "the radius vectors are: "${radi[*]}

vl=0; # cannot be a float
vu=20; # cannot be a float
declare -a vys
for ((i=0;i<NUMTESTS;i+=1));
do
  vys[${i}]=[-$((vl+RANDOM%(vu-vl))).$((RANDOM%99)),0.,0.,-1.]
done
#echo "the velocity vectors are: "${vys[*]}

declare -A parameters
idx=1;
INITIAL_TIME=$SECONDS

RUN=1;
for ((idx=0;idx<NUMTESTS;idx+=1)); do
  for plan in ${planners[@]}; do
    for known in ${knowns[@]}; do
    echo "________________________________________________________________"
    echo "Running for the $(( ${RUN} )) th time out of $num."
    echo "--------------------------------------------------------------"

    if [[ $DEBUG = "true" ]]; then
      rm /home/mavs/MAVs/results/tmp.bag
      rosclean purge -y
      parameters["/system/nloptcontrol_planner/flags/known_environment"]=${known}
      parameters["/planner/nloptcontrol_planner/misc/movingObstacles"]=${plan}
      parameters["/case/actual/obstacle/vy"]=${vys[$idx]}
      parameters["/case/actual/obstacle/radius"]=${radi[$idx]}
      start_roscore
      sleep 2
      rosparam set "/case/id" "$FOLDERNAME"test"$RUN"
      mkdir /home/mavs/MAVs/results/$FOLDERNAME/test"$RUN"
      cd /home/mavs/MAVs/results/$FOLDERNAME/test"$RUN"
      loop_entry_point
      echo "Entering: postProcess"
      cd /home/mavs/MAVs/results
      #rosbag reindex -f tmp.bag
      rosbag info tmp.bag
      python bag_to_csv.py /home/mavs/MAVs/results/tmp.bag "/home/mavs/MAVs/results/$FOLDERNAME/test"$RUN""
      julia plottingData.jl "/home/mavs/MAVs/results/$FOLDERNAME/test"$RUN"" "tmp"
      rm /home/mavs/MAVs/results/$FOLDERNAME/test"$RUN"/state.csv
      echo "Exiting: postProcess"
    else
      rm /home/mavs/MAVs/results/tmp.bag &>/dev/null
      rosclean purge -y &>/dev/null
      parameters["/system/nloptcontrol_planner/flags/known_environment"]=${known}
      parameters["/planner/nloptcontrol_planner/misc/movingObstacles"]=${plan}
      parameters["/case/actual/obstacle/vy"]=${vys[$idx]}
      parameters["/case/actual/obstacle/radius"]=${radi[$idx]}
      start_roscore &>/dev/null
      sleep 2
      rosparam set "/case/id" "$FOLDERNAME"test"$RUN" &>/dev/null
      mkdir /home/mavs/MAVs/results/$FOLDERNAME/test"$RUN"
      cd /home/mavs/MAVs/results/$FOLDERNAME/test"$RUN"
      loop_entry_point &>/dev/null
      #echo "Entering: postProcess"
      cd /home/mavs/MAVs/results
      #rosbag reindex -f tmp.bag
      rosbag info tmp.bag &>/dev/null
      python bag_to_csv.py /home/mavs/MAVs/results/tmp.bag "/home/mavs/MAVs/results/$FOLDERNAME/test"$RUN"" &>/dev/null
      julia plottingData.jl "/home/mavs/MAVs/results/$FOLDERNAME/test"$RUN"" "tmp" &>/dev/null
      rm /home/mavs/MAVs/results/$FOLDERNAME/test"$RUN"/state.csv &>/dev/null
      #echo "Exiting: postProcess"
    fi

    TIME1=$(( ($SECONDS-INITIAL_TIME) / (idx+1) * (num-(idx+1)) ))
    echo "_________________________________________________________________"
    echo "Results summary: goals attained"
    echo "--------------------------------------------------------------"
    echo "known environemt = $known_RESULTS out of $num tests."
    echo "unknown environemt = $unknown_RESULTS out of $num tests."
    echo "--------------------------------------------------------------"
    echo "Estimated time (hours, minutes, seconds) remaining is: "
    echo $(convertsecs $TIME1)
    RUN=$(( $RUN + 1 ))

    done
  done
done
