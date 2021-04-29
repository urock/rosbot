#!/bin/zsh

# Script for automated data collection

# Usage example:
# ```
# source $(catkin locate)/devel/setup.zsh
# ./colect_data.zsh
# ```
# DONE!
# kick back and wait



NUM=75
source $(catkin locate)/devel/setup.zsh
for i in $(seq 1 $NUM)
do
	python3 collect_data.py
	kill $(pidof gzserver)
done