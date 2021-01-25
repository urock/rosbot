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
# for traj in 2.5sin -1sin 3sin1.5 sin0.5 polygon
for traj in 2.5sin polygon
  do
    # for lin_vel in 0.5 2.5 5.0 7.0 10.0
    for lin_vel in 4.0
      do
        # for cir_vel in 0.5 1.5 2.5 3.5 4.0 
        for cir_vel in 1.5 
          do
            rosparam set /max_lin_vel $lin_vel
            rosparam set /max_cir_vel $cir_vel
            rosparam set /cmd_freq 10
            roslaunch plotter plotter.launch output_folder:=/traj=$traj-lin_vel=$lin_vel-cir_vel=$cir_vel-cmd_freq=10 timeout:=300 &
            roslaunch rosbot2 test_rosbot.launch traj_type:=$traj
            ResetPose
          done      
      done
  done

# for plan in /home/vytautas/MS/catkin_ws/src/rosbot/rosbot_controller/plans/triangle.txt
#   do
#     for lin_vel in 8.0
#       do
#         for cir_vel in 2.5 3.5 4.0
#           do
#             rosparam set /max_lin_vel $lin_vel
#             rosparam set /max_cir_vel $cir_vel
#             roslaunch plotter plotter.launch output_folder:=/traj=triangle-lin_vel=$lin_vel-cir_vel=$cir_vel timeout:=300 &
#             roslaunch rosbot2 test_rosbot.launch traj_type:=from_file move_plan:=$plan
#             ResetPose
#           done      
#       done
#   done
rosnode kill --all



                       ### DONE ###
#    TRAJ         lin_vel               cir_vel  
######################################################  
#   2.5sin    0.5 2.5 5.0 7 10    0.5 1.5 2.5 3.5 4.0 
#----------------------------------------------------- 
#   -1sin     0.5 2.5 5.0 7 10    0.5 1.5 2.5 3.5 4.0
#-----------------------------------------------------
#             0.5 2.5 5.0         0.5 1.5 2.5 3.5 4.0 
#   3sin1.5   7.0                 0.5 1.5
#             10.0                0.5
#----------------------------------------------------
#   sin0.5    0.5 2.5 5.0 7 10    0.5 1.5 2.5 3.5 4.0
#----------------------------------------------------
#   polygon   0.5 2.5 5.0 7 10    0.5 1.5 2.5 3.5 4.0     


#   triangle  0.5 2.5 5.0 7       0.5 1.5 2.5 3.5 4.0
#   triangle  8.0                 0.5 1.5 
#   triangle  10.0                1.5 
