#!/bin/zsh

#Example of usage:
# 1st terminal: roscore
# 2nd terminal: ./collect_dataset.zsh

TRAJECTORIES=(
	"polygon 
	1.0sin1.0
	2.0sin0.5
	3.0sin0.1
	4.0sin0.5
	5.0sin1.0
	6.0sin0.5
	7.0sin0.4
	8.0sin0.3
	9.0sin0.2
	10.0sin0.1
	6.0sin2.0
	7.0sin2.5
	8.0sin3.0
	9.0sin2.5
	10.0sin2.0
	reverse_1.0sin1.0
	reverse_2.0sin0.5 
	reverse_3.0sin0.1 
	reverse_4.0sin0.5 
	reverse_5.0sin1.0 
	reverse_6.0sin0.5 
	reverse_7.0sin0.4
	reverse_8.0sin0.3
	reverse_9.0sin0.2
	reverse_10.0sin0.1
	reverse_6.0sin2.0
	reverse_7.0sin2.5
	reverse_8.0sin3.0
	reverse_9.0sin2.5
	reverse_10.0sin2.0
	1.0spiral1.0
	2.0spiral1.0
	3.0spiral1.0
	4.0spiral1.0
	5.0spiral1.0
	6.0spiral1.0
	7.0spiral1.0
	8.0spiral1.0
	9.0spiral1.0
	10.0spiral1.0
	-1.0spiral1.0
	-2.0spiral1.0
	-3.0spiral1.0
	-4.0spiral1.0
	-5.0spiral1.0
	-6.0spiral1.0
	-7.0spiral1.0
	-8.0spiral1.0
	-9.0spiral1.0
	-10.0spiral1.0"
)

MAX_V_LIST=(
	"0.5
	1.0
	1.5
	2.0
	2.5
	3.0
	3.5"
)

MAX_W_LIST=(
	"0.5
	1.0
	1.5
	2.0
	2.5
	3.0
	3.5"
)

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


source $(catkin locate)/devel/setup.zsh

roslaunch rosbot_controller run_simulation.launch rviz:=true &
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