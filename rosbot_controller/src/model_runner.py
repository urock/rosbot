#!/usr/bin/env python
# license removed for brevity
import rospy
import os
import roslib
import tf
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from tf.transformations import quaternion_from_euler
from modules.rosbot import Rosbot, RobotState, RobotControl


class ModelRunner():
    """

    """
    
    def __init__(self, node_name): 
        self.node_name = node_name
        rospy.init_node(self.node_name, anonymous=True)
        self.robot_frame = rospy.get_param('~robot_frame')
        self.cmd_topic = rospy.get_param('~cmd_topic')
        self.odom_frame = 'odom'

        self.robot = Rosbot()
        self.model_state = RobotState()   

        self.tf_br = tf.TransformBroadcaster()
        
        self.cmd_freq = 20.0 # Hz       
        self.dt = 1.0 / self.cmd_freq 
        self.rate = rospy.Rate(self.cmd_freq) 

        self.control_vector = RobotControl()
        self.cmd_sub = rospy.Subscriber(self.cmd_topic, Twist, self.command_callback)
        self.last_timestamp = rospy.Time.now().to_sec()


    def broadcast_model_tf(self, state):
        pose = (state.x, state.y, 0.0)
        orient = tf.transformations.quaternion_from_euler(0, 0, state.yaw)
        
        self.tf_br.sendTransform(pose, orient,
                     rospy.Time.now(),
                     self.robot_frame,
                     self.odom_frame)        

    def command_callback(self, msg):      
        self.control_vector = RobotControl(msg.linear.x, msg.angular.z)
        # rospy.logerr(self.robot_frame + ": " + self.control_vector.to_str())


    def print_state(self, state):
        rospy.loginfo(state.to_str())

    def run(self):
       
        while not rospy.is_shutdown():    
            current_timestamp = rospy.Time.now().to_sec()
            dt = current_timestamp - self.last_timestamp 

            self.model_state = self.robot.update_state_by_model(self.control_vector, dt)
            # rospy.logerr(self.robot_frame + ": " + self.model_state.to_str())
            self.broadcast_model_tf(self.model_state)

            self.last_timestamp = current_timestamp
            
            self.rate.sleep()    


def main():
    robot_model = ModelRunner('robot_model')
    robot_model.run()

if __name__ == '__main__':
    main()
