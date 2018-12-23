#!/bin/bash

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
  roslaunch system sweepB.launch &
  roslaunch_PID=$!
}
wait_for_fixed_execution_time () {
  DURATION=1000
  START_TIME=$SECONDS
  until (($(( SECONDS - START_TIME )) > "$DURATION")) ; do sleep 1; done
}
loop_entry_point () {
  run_launchfile
  wait_for_fixed_execution_time
  kill_roslaunch_pid
}

declare -A parameters
#arr+=( ["key2"]=val2 ["key3"]=val3 )

for ((i=0;i<2;i+=1)); do echo "cases[${i}]=${i}"; done
for ((i=0;i<2;i+=1)); do echo "speeds[${i}]=[${i}]"; done


#start_roscore  # NOTE this may need to be stopped in the launch script. Can that create an issue?
for ((i=0;i<2;i+=1));
  echo "$i" "${cases[i]}"
  parameters["/case/id"]="${cases[i]}"
  parameters["/case/actual/obstacle/vy"]="${speeds[i]}"
  # in future add nested loops here and call entry_point in the innermost loop
#  loop_entry_point
done
#kill_roscore_pid
