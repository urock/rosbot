#!/bin/bash

image_name=gazebo-control-image
container_name=gazebo-control

docker run -it -d --privileged --net=host \
      --name $container_name \
      --runtime=nvidia \
      --gpus=all \
      -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
      -v ${PWD}/../:/home/user/catkin_ws/src:rw \
      -e DISPLAY=$DISPLAY \
      -e NVIDIA_VISIBLE_DEVICES="all" \
      -e NVIDIA_DRIVER_CAPABILITIES="all" \
      -e ROS_HOSTNAME="localhost" \
      -e ROS_MASTER_URI="http://localhost:11311" \
      -e QT_X11_NO_MITSHM=1 $image_name zsh 
