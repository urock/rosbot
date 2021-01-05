#!/bin/zsh

while [[ "$#" -gt 0 ]]; do
    case $1 in
        -p|--path) path_="$2"; shift ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

echo source $path_/devel/setup.zsh
source $path_/devel/setup.zsh
roslaunch rosbot2 urock_system.launch traj_type:=sin