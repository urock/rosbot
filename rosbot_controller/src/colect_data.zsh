#!/bin/zsh
NUM=35
source $(catkin locate)/devel/setup.zsh
for i in $(seq 1 $NUM)
do
	python3 collect_data.py
	kill $(pidof gzserver)
done