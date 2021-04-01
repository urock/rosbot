#!/bin/bash

image_name=rosbot-control
container_name=control

docker run -it -d --privileged --net=host \
      --name $container_name \
      --runtime=nvidia \
      --gpus=all \
      -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
      -v ${PWD}/../:/home/user/catkin_ws/src:rw \
      -e DISPLAY=$DISPLAY \
      -e NVIDIA_VISIBLE_DEVICES="all" \
      -e NVIDIA_DRIVER_CAPABILITIES="all" \
      -e QT_X11_NO_MITSHM=1 $image_name zsh
