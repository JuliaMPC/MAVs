#!/bin/bash

# test script to programatically make folders

for i in {1..5}; do
  mkdir /home/mavs/MAVs/results/sweepA/test"$i"
  cd /home/mavs/MAVs/results;
  python bag_to_csv.py tmp.bag "sweepA/test$i"
  julia plottingData.jl "sweepA/test$i" "tmp"
  rm /home/mavs/MAVs/results/sweepA/test$i/control.csv
  rm /home/mavs/MAVs/results/sweepA/test$i/state.csv
  rm /home/mavs/MAVs/results/sweepA/test$i/nloptcontrol_plannercontrol.csv
done
