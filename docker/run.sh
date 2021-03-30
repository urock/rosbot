#!/bin/bash

image_name=rosbot-control
container_name=control

xhost +

docker run -it  \
      --privileged  \
      --net=host \
	    --name $container_name \
      -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
      -v ~/work/rosbot/catkin_ws/:/home/user/catkin_ws:rw \
      -v ~/.ssh:/root/ssh \
      -v ~/.gazebo:/home/user/.gazebo/:rw \
      -e DISPLAY=$DISPLAY \
      -e NVIDIA_VISIBLE_DEVICES="all" \
      -e NVIDIA_DRIVER_CAPABILITIES="all" \
      -e QT_X11_NO_MITSHM=1 $image_name bash\
      --gpus=all 

echo "Container Built"
