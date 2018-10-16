#!/bin/sh

# Runs a docker container with the image created by build.sh
xhost +local:root

until sudo nvidia-docker ps
do
    echo "Waiting for docker server"
    sleep 1
done

XSOCK=/tmp/.X11-unix
XAUTH=/home/$USER/.Xauthority
SHARED_DIR=/home/mavs/MAVs/shared_dir
HOST_DIR=/home/$USER/MAVs/shared_dir
SRC_CONTAINER=/home/mavs/MAVs/ros/src
SRC_HOST=/home/$USER/MAVs/ros/src

echo "Shared directory: ${HOST_DIR}"

nvidia-docker run \
    --user mavs \
    -it --rm \
    --volume=$XSOCK:$XSOCK:rw \
    --volume=$XAUTH:$XAUTH:rw \
    --volume=$HOST_DIR:$SHARED_DIR:rw \
    --volume=$SRC_HOST:$SRC_CONTAINER:rw \
    --env="XAUTHORITY=${XAUTH}" \
    --env="DISPLAY=${DISPLAY}" \
    --privileged -v /dev/bus/usb:/dev/bus/usb \
    --net=host \
    mavs

xhost -local:root