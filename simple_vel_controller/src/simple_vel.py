#!/usr/bin/env python
# license removed for brevity
import rospy
import os
import roslib
import tf
import time
from time import gmtime, strftime
from pathlib import Path
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Pose, PoseStamped, Twist
from geometry_msgs.msg import Quaternion
import math
import numpy as np
from tf.transformations import quaternion_from_euler

from nav_msgs.msg import Path

from modules.rosbot import Rosbot, RobotState, Goal

# from visualization_msgs.msg import Marker


class TrajKeeper():
    """
    Gived Robot and Controll module this modules tries to follow given path 
    Trajectory should be defined as array of (x, y, yaw ?) points
    """
    

    def __init__(self, node_name): 
        self.node_name = node_name
        rospy.init_node(self.node_name, anonymous=True)

        rospy.loginfo("trajectory_keeper")

        self.robot = Rosbot()

        self.tf_listener = tf.TransformListener()
        self.tf_br = tf.TransformBroadcaster()

        self.cmd_freq = 10 # Hz        

        self.odom_state = RobotState()     # x, y, yaw, vx, vy,  w
        self.current_goal = Goal()               # x, y 
        self.control = (0, 0)               # v, w

        self.goal_queue = []

        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)
        self.path_sub = rospy.Subscriber("/path", Path, self.path_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        
        self.rate = rospy.Rate(self.cmd_freq)


    def set_robot_odom_state(self):
        src_frame = 'odom'
        dst_frame = 'base_link'
        try:
            (coord,orient) = self.tf_listener.lookupTransform(src_frame, dst_frame, rospy.Time())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Error in lookup transform from {} to {}".format(src_frame, dst_frame))    
            return False

        x, y = coord[0], coord[1]
        yaw = tf.transformations.euler_from_quaternion(orient)[2]

        vx = (x   - self.odom_state.x) * self.cmd_freq
        vy = (y   - self.odom_state.y) * self.cmd_freq
        w  = (yaw - self.odom_state.yaw) * self.cmd_freq

        self.odom_state = RobotState(x, y, yaw, vx, vy, w)

        self.robot.set_odom_state(self.odom_state)

        return True

    def send_tf(self, state):
        pose = (state.x, state.y, 0.0)
        orient = tf.transformations.quaternion_from_euler(0, 0, state.yaw)
        
        self.tf_br.sendTransform(pose, orient,
                     rospy.Time.now(),
                     "model_link",
                     "odom")

    def print_state(self, st):
        rospy.loginfo(" x -> {:.2f}, y -> {:.2f}, yaw -> {:.2f}, vx -> {:.2f}, vy -> {:.2f}, w -> {:.2f}".format(
                        st.x, st.y, st.yaw, st.vx, st.vy, st.w))

    def goal_callback(self, msg):
        x, y = msg.pose.position.x, msg.pose.position.y            
        self.goal_queue.append(Goal(x, y))
        rospy.logwarn("added goal to queue: x -> {:.2f}, y -> {:.2f}".format(x, y))
        
    def path_callback(self, msg):
        for p in msg.poses:
            x, y = p.pose.position.x, p.pose.position.y
            self.goal_queue.append(Goal(x, y))
    
    def publish_twist(self, v, w):
        """
        :param v: linear velocity, ``float``
        :param w: angular velocity, ``float``
        """
        twist_cmd = Twist()
        twist_cmd.linear.x = v
        twist_cmd.linear.y = 0
        twist_cmd.linear.z = 0
        twist_cmd.angular.x = 0
        twist_cmd.angular.y = 0
        twist_cmd.angular.z = w
        self.cmd_pub.publish(twist_cmd)

      

    def run(self):
        while not rospy.is_shutdown():
            if not self.set_robot_odom_state():
                continue
            # check if new goal should be selected and calc control
            if self.robot.goal_reached(self.current_goal):
                if self.goal_queue:
                    self.current_goal = self.goal_queue.pop(0)
                    rospy.logerr("new current_goal: x -> {:.2f}, y -> {:.2f}".
                    format(self.current_goal.x, self.current_goal.y))
                else:
                    # end of trajectory
                    self.publish_twist(0, 0)    
                    self.rate.sleep()
                    continue

            v, w = self.robot.calculate_contol(self.current_goal)
            self.publish_twist(v, w)

            model_state = self.robot.update_model_state(v, w, self.cmd_freq)
            self.send_tf(model_state)

            self.rate.sleep()

def main():
    trajectory_keeper = TrajKeeper('trajectory_keeper')
    trajectory_keeper.run()

if __name__ == '__main__':
    main()
