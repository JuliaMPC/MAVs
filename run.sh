#!/bin/sh

# Runs a docker container with the image created by build.sh
until docker ps
do
    echo "Waiting for docker server"
    sleep 1
done

XSOCK=/tmp/.X11-unix
XAUTH=/home/$USER/.Xauthority
SHARED_CONTAINER=/home/mavs/MAVs/shared_dir
SHARED_HOST="$(pwd)"/shared_dir
SRC_CONTAINER=/home/mavs/MAVs/ros/src
SRC_HOST="$(pwd)"/ros/src

echo "Shared directory: ${SHARED_CONTAINER}"

docker run \
    --user mavs \
    -it --rm \
    --volume=$XSOCK:$XSOCK:rw \
    --volume=$XAUTH:$XAUTH:rw \
    --volume=$SHARED_HOST:$SHARED_CONTAINER:rw \
    --volume=$SRC_HOST:$SRC_CONTAINER:rw \
    --env="XAUTHORITY=${XAUTH}" \
    --env="DISPLAY=${DISPLAY}" \
    --privileged -v /dev/bus/usb:/dev/bus/usb \
    --net=host \
    mavs
