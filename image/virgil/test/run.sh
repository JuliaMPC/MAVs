#!/bin/sh

XSOCK=/tmp/.X11-unix
XAUTH=/home/$USER/.Xauthority
SHARED_DIR=/home/$USER/shared_dir
HOST_DIR=/home/$USER/shared_dir

if [ "$1" = "" ]
then
    # Create Shared Folder
    mkdir -p $SHARED_DIR
else
    HOST_DIR=$1
fi
echo "Shared directory: ${HOST_DIR}"

nvidia-docker run \
    -it --rm \
    --volume=$XSOCK:$XSOCK:rw \
    --volume=$XAUTH:$XAUTH:rw \
    --volume=$HOST_DIR:$SHARED_DIR:rw \
    --env="XAUTHORITY=${XAUTH}" \
    --env="DISPLAY=${DISPLAY}" \
    -u mavs \
    --privileged -v /dev/bus/usb:/dev/bus/usb \
    --net=host \
    mavs
