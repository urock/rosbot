import os
import rospy
import rosservice
import numpy as np
import rosnode
from geometry_msgs.msg import PoseStamped, Twist
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState



os.popen("roslaunch rosbot_controller run_simulation.launch rviz:=true")
rospy.sleep(7)
T = 1 # sec
rospy.init_node("data_collector", anonymous=True)
cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)


def ResetPose():
    state_msg = ModelState()
    state_msg.model_name = 'rosbot'
    state_msg.reference_frame = 'world'
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state( state_msg )

    except:
        pass

    state_msg = ModelState()
    state_msg.model_name = 'rosbot'

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state( state_msg )

    except:
        pass




def run_control_gen(
	output_folder,
	Tmax,
	v_max,
	w_max,
	a_lin,
	a_ang,
	period_lin,
	period_ang
	):

	os.popen("roslaunch rosbot_controller spawn_kinematic_model.launch")
	rospy.sleep(5)
	os.popen(
		f"roslaunch rosbot_controller control_gen.launch output_folder:=/{output_folder} Tmax:={Tmax} v_max:={v_max} w_max:={w_max} a_lin:={a_lin} a_ang:={a_ang} period_lin:={period_lin} period_ang:={period_ang} v_min:={-v_max} w_min:={-w_max}"
		)

	rospy.sleep(3)
	while "/logger" in rosnode.get_node_names():
		rospy.sleep(T)

	stop = Twist()
	cmd_pub.publish(stop)	
	return





for _ in range(50):
	Tmax = 50
	v_max = round(np.random.uniform(low=0.2, high=1.5), 2) # TODO find high=0.5
	w_max = round(np.random.uniform(low=0.2, high=1.5), 2)
	a_lin = round(np.random.uniform(low=0.1, high=0.5), 2) # TODO find high=0.5
	a_ang = round(np.random.uniform(low=-0.2, high=0.2), 2)
	period_lin = np.random.randint(5, Tmax)
	period_ang = np.random.randint(5, Tmax) 

	output_folder = "control_gen_Tmax={}_v_max={}_w_max={}_a_lin={}_a_ang={}_per_lin={}_per_ang={}".format(Tmax, v_max, w_max, a_lin, a_ang, period_lin, period_ang)

	run_control_gen(output_folder, Tmax, v_max, w_max, a_lin, a_ang, period_lin, period_ang)
	ResetPose()


os.popen("rosnode kill -a")