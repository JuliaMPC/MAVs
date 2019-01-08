#!/bin/bash

cases=("casea" "caseb")

wait_for_roscore_to_initialize () {
  until rostopic list ; do sleep 1; done
}
wait_for_roslaunch_pid_to_stop () {
  while kill -s 0 "$roslaunch_PID"; do
    sleep 2
  done
}
kill_roslaunch_pid () {
  kill -INT $roslaunch_PID
  wait_for_roslaunch_pid_to_stop
  sleep 3
}
kill_roscore_pid () {
  kill -INT $roscore_PID
  sleep 2
}
update_dynamic_ros_parameters () {
  for key in "${!parameters[@]}"; do
    rosparam set $key ${parameters[$key]}
  done
}
start_roscore () {
  roscore &
  roscore_PID=$!
  wait_for_roscore_to_initialize
}
run_launchfile () {
  update_dynamic_ros_parameters
  roslaunch system sweepA.launch &
  roslaunch_PID=$!
  sleep 4
}
wait_for_program () {
  DURATION=2000
  START_TIME=$SECONDS
  shutdown=0;
until (($(( SECONDS - START_TIME )) > "$DURATION")) || [[ $shutdown -eq 1 ]]; do
  sleep 1;
  if [[ `rosparam get "system/flags/done"` = "true" ]]; then
    shutdown=1;
    sleep 45  # assuming that this is enough time, but there is not check to make sure that everything has shutdown.
  fi
  done
}

loop_entry_point () {
  run_launchfile
  wait_for_program
  kill_roslaunch_pid
}

declare -A parameters
#arr+=( ["key2"]=val2 ["key3"]=val3 )
start_roscore
for case in ${cases[@]}; do
  parameters["/case/id"]=${case}
  # in future add nested loops here and call entry_point in the innermost loop
  loop_entry_point
done
kill_roscore_pid
