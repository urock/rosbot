#!/usr/bin/env python3

import rospy

from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

class GazeboState:
    def __init__(self, model, reference_frame):
        self.set_state_proxy = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        self.default_state = ModelState()
        self.default_state.model_name = model 
        self.default_state.reference_frame = reference_frame 


    def reset(self):
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_state_proxy(self.default_state)

