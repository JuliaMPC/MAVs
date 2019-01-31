#!/bin/bash
# $1 is the bag name, 2 is the folder name, 3 is the YAML case name
BAGNAME="$1"
FOLDERNAME="$2"
CASE="$3"

# EX: $./postP.sh "test.bag" "test" "s8"
#shutdown=0;
#until [[ $shutdown -eq 1 ]]; do
#  sleep 1;
  #if ! rostopic list ; then
#  if [[ `rosparam get "system/flags/done"` = "true" ]]; then
#    echo "starting postP!"
    rm -rf /home/mavs/MAVs/results/tests/$FOLDERNAME/*
    rmdir /home/mavs/MAVs/results/tests/$FOLDERNAME
    mkdir /home/mavs/MAVs/results/tests/$FOLDERNAME
    cd /home/mavs/MAVs/results
    python bag_to_csv.py $BAGNAME "/home/mavs/MAVs/results/tests/$FOLDERNAME"
    cd /home/mavs/MAVs/results/
    julia plottingData.jl "/home/mavs/MAVs/results/tests/$FOLDERNAME" "$CASE"
    rm /home/mavs/MAVs/results/tests/$FOLDERNAME/control.csv
    rm /home/mavs/MAVs/results/tests/$FOLDERNAME/state.csv
    rm /home/mavs/MAVs/results/tests/$FOLDERNAME/nloptcontrol_plannercontrol.csv
#    sleep 2;
#    echo `rosparam set "system/shutdown/flags/postProcess" true`
#    shutdown=1;
#  fi
#done
