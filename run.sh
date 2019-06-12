#!/bin/sh

# Runs a docker container with the image created by build.sh
until nvidia-docker ps
do
    echo "Waiting for docker server"
    sleep 1
done

XSOCK=/tmp/.X11-unix
XAUTH=/home/$USER/.Xauthority

RESULTS_CONTAINER=/home/mavs/MAVs/results
RESULTS_HOST="$(pwd)"/results

SRC_CONTAINER=/home/mavs/MAVs/ros/src
SRC_HOST="$(pwd)"/ros/src

TRAJ_CONTAINER=/home/mavs/.julia/v0.6/MichiganAutonomousVehicles
TRAJ_HOST="$(pwd)"/MichiganAutonomousVehicles

echo "Results directory: ${RESULTS_CONTAINER}"

xhost local:root

nvidia-docker run \
    --name mavs_container \
    --user mavs \
    -it --rm \
    --volume=$XSOCK:$XSOCK:rw \
    --volume=$XAUTH:$XAUTH:rw \
    --volume=$RESULTS_HOST:$RESULTS_CONTAINER:rw \
    --volume=$SRC_HOST:$SRC_CONTAINER:rw \
    --volume=$TRAJ_HOST:$TRAJ_CONTAINER:rw \
    --env="XAUTHORITY=${XAUTH}" \
    --env="DISPLAY=${DISPLAY}" \
    --privileged -v /dev/bus/usb:/dev/bus/usb \
    --net=host \
    mavs
