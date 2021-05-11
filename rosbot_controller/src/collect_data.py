import os
import numpy as np
import rospy
import rosservice
import rosnode
from geometry_msgs.msg import Twist, ModelState
from gazebo_msgs.srv import SetModelState


def ResetPose():
    """
    Teleports the rosbot to the origin
    """
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

    """
    Launch the control_generator,
    and wait for the end of the trajectory 
    """

    os.popen("roslaunch rosbot_controller spawn_kinematic_model.launch")
    rospy.sleep(5)
    os.popen(
        f"roslaunch rosbot_controller control_gen.launch output_folder:=/{output_folder} Tmax:={Tmax} v_max:={v_max} w_max:={w_max} a_lin:={a_lin} a_ang:={a_ang} period_lin:={period_lin} period_ang:={period_ang} v_min:={0} w_min:={0}"
        )

    rospy.sleep(3)
    # Wait while the logger is working
    while "/logger" in rosnode.get_node_names():
        rospy.sleep(1)

    # Stop rosbot (just in case)
    cmd_pub.publish(Twist())    
    return




# UPDATE_RATE = np.random.randint(10, 34)
DT_MAX = 0.2            # maximum time delta
DT_MIN = 0.03           # minimal time delta
Tmax = 50               # control_generator running time
UPDATE_RATE = np.random.uniform(1/DT_MIN, 1/DT_MAX) # sample rosbot update_rate

# spawn rosbot and kinetic model
os.popen("roslaunch rosbot_controller run_simulation.launch rviz:=false update_rate:={} cmd_freq:={}".format(UPDATE_RATE, UPDATE_RATE))
rospy.sleep(7)
rospy.init_node("data_collector", anonymous=True)
cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

# Sample trajectory parameters
v_max = round(np.random.uniform(low=0.5, high=1.5), 2)    # max linear velocity
w_max = round(np.random.uniform(low=1.5, high=2.5), 2)    # max angular velocity
a_lin = round(np.random.uniform(low=-0.5, high=0.5), 4)   # linear acceleration
a_ang = round(np.random.uniform(low=-0.5, high=0.5), 4)   # angular acceleration
period_lin = np.random.randint(10, int(Tmax/2))           # linear velocity changing period
period_ang = np.random.randint(10, int(Tmax/2))           # angular velocity changing period

# Composing the first trajectory folder name
output_folder = "control_gen_dt_{}_Tmax={}_v_max={}_w_max={}_a_lin={}_a_ang={}_per_lin={}_per_ang={}".format(
    round(1/UPDATE_RATE, 3),
    Tmax,
    v_max,
    w_max,
    a_lin,
    a_ang,
    period_lin,
    period_ang
)

# Collect the first trajectory
run_control_gen(output_folder, Tmax, v_max, w_max, a_lin, a_ang, period_lin, period_ang)
ResetPose() # teleport the rosbot to the origin

# update angular acceleration, so second trajectory will be mirrored
a_ang = -a_ang
# Composing the second trajectory folder name
output_folder = "control_gen_dt_{}_Tmax={}_v_max={}_w_max={}_a_lin={}_a_ang={}_per_lin={}_per_ang={}".format(
    round(1/UPDATE_RATE, 3),
    Tmax,
    v_max,
    w_max,
    a_lin,
    a_ang,
    period_lin,
    period_ang
)

# Collect the second trajectory
run_control_gen(output_folder, Tmax, v_max, w_max, a_lin, a_ang, period_lin, period_ang)
ResetPose() # teleport the rosbot to the origin

# kill all ROS nodes
os.popen("rosnode kill -a")
# Completing the script
