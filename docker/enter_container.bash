#!/bin/bash

container_name=rosbot-container

xhost +local:root

docker start $container_name

docker exec -it $container_name bash