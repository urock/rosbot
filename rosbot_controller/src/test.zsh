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
roslaunch rosbot2 urock_system.launch rviz:=true gui:=false &
for traj in 2.5sin -1sin 3sin1.5 sin0.1 polygon
  do
    for lin_vel in 0.5 2.5 5.0 7.0 10.0 15.0
      do
        for cir_vel in 0.5 2.5 5.0 7.0 10.0 15.0
          do
            rosparam set /max_lin_vel $lin_vel
            rosparam set /max_cir_vel $cir_vel
            roslaunch plotter plotter.launch output_folder:=/$traj/$lin_vel-$cir_vel &
            roslaunch rosbot2 test_rosbot.launch traj_type:=$traj
            ResetPose
          done      
      done
  done
rosnode kill --all