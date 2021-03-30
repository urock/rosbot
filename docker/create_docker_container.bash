#!/bin/bash

image_name=rosbot-melodic 
container_name=rosbot-container

xhost +local:root
xhost +

docker run -it --privileged --name $container_name \
            --env="DISPLAY" \
            --env="QT_X11_NO_MITSHM=1" \
            --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
            -v ~/work/rosbot/:/home/user/catkin_ws/src:rw \
            -v ~/.ssh:/root/ssh \
            -v ~/.gazebo:/home/user/.gazebo/:rw \
            $image_name bash

exit $?
