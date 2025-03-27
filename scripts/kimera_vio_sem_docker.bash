#!/bin/bash

# Allow X server connection
# xhost +local:root
docker run -it \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/home/art5gpc6/datasets/:/datasets/" \
    --name kimera_vio_sem kimera_vio_sem_ros
# Disallow X server connection
# xhost -local:root
