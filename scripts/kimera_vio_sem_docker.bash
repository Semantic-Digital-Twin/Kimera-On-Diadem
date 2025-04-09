#!/bin/bash

# Allow X server connection
# xhost +local:root
docker run --memory=8g --memory-swap=8g -it \
    --privileged \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --device=/dev/bus/usb \
    -v /dev/bus/usb:/dev/bus/usb \
    -v /etc/udev/rules.d:/etc/udev/rules.d \
    -v /lib/udev:/lib/udev \
    -v /run/udev:/run/udev \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/home/art5gpc6/datasets/:/datasets/" \
    --name kimera_vio_sem kimera_vio_sem_ros
# Disallow X server connection
# xhost -local:root

# Use --rm if you wish for the container to be removed once you exit it
