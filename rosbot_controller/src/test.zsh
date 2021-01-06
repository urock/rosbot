#!/bin/zsh

ResetPose() {
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
   x: 0.0
   y: 0.0
   z: 0.0
 angular:
   x: 0.0
   y: 0.0
   z: 0.0"

rosservice call /gazebo/set_model_state '{model_state: { model_name: rosbot, pose: { position: { x: 0.0, y: 0.0 ,z: 0.0 }, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0 } }, twist: { linear: {x: 0.0 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 0.0 } } , reference_frame: world } }'
}


while [[ "$#" -gt 0 ]]; do
    case $1 in
        -p|--path) path_="$2"; shift ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

echo source $path_/devel/setup.zsh
source $path_/devel/setup.zsh
roslaunch rosbot2 urock_system.launch &
roslaunch plotter plotter.launch track_time:=true &
roslaunch rosbot_controller folow_path.launch traj_type:=sin
ResetPose
roslaunch plotter plotter.launch track_time:=true &
roslaunch rosbot_controller folow_path.launch traj_type:=polygon
#ResetPose
#roslaunch rosbot_controller folow_path.launch traj_type:=sin