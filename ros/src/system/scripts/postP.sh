#!/bin/bash
# $1 is the bag name, 2 is the folder name, 3 is the YAML case name
BAGNAME="$1"
FOLDERNAME="$2"
CASE="$3"

# EX: $./postP.sh "test.bag" "test" "s8"

rm -rf /home/mavs/MAVs/results/tests/$FOLDERNAME/*
rmdir /home/mavs/MAVs/results/tests/$FOLDERNAME
mkdir /home/mavs/MAVs/results/tests/$FOLDERNAME
cd /home/mavs/MAVs/results
python bag_to_csv.py $BAGNAME "/home/mavs/MAVs/results/tests/$FOLDERNAME"
julia plottingData.jl "/home/mavs/MAVs/results/tests/$FOLDERNAME" "$CASE"
rm /home/mavs/MAVs/results/tests/$FOLDERNAME/control.csv
rm /home/mavs/MAVs/results/tests/$FOLDERNAME/state.csv
rm /home/mavs/MAVs/results/tests/$FOLDERNAME/nloptcontrol_plannercontrol.csv
