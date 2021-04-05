#!/bin/zsh

#Example of usage:
# 1st terminal: roscore
# 2nd terminal: ./test.zsh -t="1sin0.5 -3.8sin2.5" -v="1.0 2.0 3.0 4.0 5.0" -w="0.5 1.0 2.0 3.0 4.0"


ResetPose() {
  # Reset rosbot pose to x=0 y=0
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


for i in "$@"
  do
    case $i in
        # key for angular trajectories
        -t=*|--traj=*)
        TRAJECTORIES="${i#*=}"
        ;;

        # key for linear velocities
        -v=*|--velocity=*)
        MAX_V_LIST="${i#*=}"
        ;;

        # key for angular velocities
        -w=*|--ang_vel=*)
        MAX_W_LIST="${i#*=}"
        ;;

    esac
  done

source $(catkin locate)/devel/setup.zsh

roslaunch rosbot2 spawn_rosbot.launch rviz:=true gui:=false &
for traj in $(echo $TRAJECTORIES | tr " " " ")
  do
    for v in $(echo $MAX_V_LIST | tr " " " ")
      do
        for w in $(echo $MAX_W_LIST | tr " " " ")
          do
            rosparam set /max_v $v
            rosparam set /max_w $w
            roslaunch logger logger.launch output_folder:=/traj=$traj-max_v=$v-max_w=$w timeout:=300 &
            roslaunch rosbot_controller spawn_kinematic_model.launch &
            roslaunch rosbot_controller follow_path.launch traj_type:=$traj
            ResetPose
          done      
      done
  done
rosnode kill -all