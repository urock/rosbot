#!/bin/zsh

#Example of usage:
# 1st terminal: roscore
# 2st terminal

#  ./gradient_descent_test.zsh -n=1 -p=/home/vytautas/Desktop/control_test.txt -o=/home/vytautas/MS/catkin_ws/src/rosbot/logger/output_data
# -n= how many times is repeated
# -p= path to file with control
# -o= path to logger output_data (or just folder with collected data)
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
        -n=*|--num=*)
        NUM_ITER="${i#*=}"
        ;;

        # key for linear velocities
        -p=*|--path=*)
        TRAJ_PATH="${i#*=}"
        ;;

        # key for linear velocities
        -o=*|--out_data=*)
        OUT_DATA="${i#*=}"
        ;;

    esac
  done

source $(catkin locate)/devel/setup.zsh
roslaunch rosbot2 spawn_rosbot.launch rviz:=true gui:=false &
for i in $(seq 1 $NUM_ITER)
  do
    roslaunch logger logger.launch output_folder:=/num=$i&
    roslaunch rosbot_controller spawn_kinematic_model.launch&
    python3 contol_generator.py -file_path $TRAJ_PATH
    rosnode kill logger &
    rosnode kill control_generator &
    ResetPose
  done
rosrun logger draw_states.py $OUT_DATA $NUM_ITER
rosnode kill -all