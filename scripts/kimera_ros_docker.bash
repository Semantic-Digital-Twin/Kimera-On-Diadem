#!/bin/bash

# Allow X server connection
xhost +local:root
# --rm to remove container after it is stopped
# docker run -it --rm \
docker run -it \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/home/user/datasets/:/home/user/datasets/" \
    -v ~/sharedWDocker:/home/user \
    kimera_vio_ros
# Disallow X server connection
xhost -local:root
