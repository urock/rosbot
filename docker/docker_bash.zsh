#!/bin/zsh

image_name=rosbot-melodic 
container_name=rosbot-docker

xhost +
timeout=1200

# check if there is flag to delete container after exit
docker run -it --privileged --name $container_name \
            -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
            -v ~/work/rosbot/catkin_ws/:/home/user/catkin_ws:rw \
            -v ~/.ssh:/root/ssh \
            -v ~/.gazebo:/home/user/.gazebo/:rw \
            --stop-timeout $timeout \
            -e DISPLAY=$DISPLAY $image_name bash

docker rm $container_name

exit $?