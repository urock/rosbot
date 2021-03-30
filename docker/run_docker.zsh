#!/bin/zsh

image_name=rosbot-melodic 
container_name=rosbot-docker

xhost +
timeout=1200

# check if there is flag to delete container after exit
docker run -it --rm --privileged --net=host \
	    --name $container_name \
      -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
      -v ~/work/rosbot/catkin_ws/:/home/user/catkin_ws:rw \
      -v ~/.ssh:/root/ssh \
      -v ~/.gazebo:/home/user/.gazebo/:rw \
      -e DISPLAY=$DISPLAY \
      --gpus=0 \
      -e NVIDIA_VISIBLE_DEVICES="all" \
      -e NVIDIA_DRIVER_CAPABILITIES="all" \
      -e QT_X11_NO_MITSHM=1 $image_name bash

docker rm $container_name

exit $?
