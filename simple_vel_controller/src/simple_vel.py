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
from tf.transformations import quaternion_from_euler

from std_msgs.msg import Float64
from std_msgs.msg import String

# from visualization_msgs.msg import Marker


class Bot():

    

    def __init__(self, node_name): 
        self.node_name = node_name
        rospy.init_node(self.node_name, anonymous=True)

        rospy.loginfo("Bot init")

        self.v_max = 0.4
        self.w_max = 0.4

        self.xy_margin_2 = 0.01

        self.tf_listener = tf.TransformListener()

        self.cmd_freq = 10 # Hz        

        self.state = (0, 0, 0, 0, 0, 0)     # x, y, yaw, vx, vy,  w
        self.goal = (0, 0, 0)               # x, y, yaw 
        self.control = (0, 0)               # v, w

        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        
        self.rate = rospy.Rate(self.cmd_freq)


    def get_current_state(self):
        src_frame = 'odom'
        dst_frame = 'base_link'
        try:
            (coord,orient) = self.tf_listener.lookupTransform(src_frame, dst_frame, rospy.Time())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Error in lookup transform from {} to {}".format(src_frame, dst_frame))    
            return

        x, y = coord[0], coord[1]
        yaw = tf.transformations.euler_from_quaternion(orient)[2]

        vx = (x   - self.state[0]) * self.cmd_freq
        vy = (y   - self.state[1]) * self.cmd_freq
        w  = (yaw - self.state[2]) * self.cmd_freq

        self.state = (x, y, yaw, vx, vy, w)

    def print_state(self):
        rospy.loginfo(" x -> {:.2f}, y -> {:.2f}, yaw -> {:.2f}, vx -> {:.2f}, vy -> {:.2f}, w -> {:.2f}".format(
                        self.state[0], self.state[1], self.state[2], self.state[3], self.state[4], self.state[5]))

    def goal_callback(self, msg):
        x, y = msg.pose.position.x, msg.pose.position.y
        azim_goal = math.atan2((y - self.state[1]),(x - self.state[0]))
        self.goal = (x, y, azim_goal)
        rospy.logwarn("goal_callback: x_goal -> {:.2f}, goal_y -> {:.2f}".format(self.goal[0], self.goal[1]))
        
    
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


    def calc_control(self):
        rho = (self.goal[0] - self.state[0])**2 + (self.goal[1] - self.state[1])**2
        alpha = self.goal[2] - self.state[2]

        if rho < self.xy_margin_2: 
            self.control = (0.0, 0.0)
            return

        v = self.v_max * math.tanh(rho) * math.cos(alpha)
        w = self.w_max * alpha + math.tanh(rho)*math.sin(alpha)*math.cos(alpha)/rho

        self.control = (v, w)

    

    def run(self):
        while not rospy.is_shutdown():
            self.get_current_state()
            self.calc_control()
            self.publish_twist(self.control[0], self.control[1])
            self.print_state()
            self.rate.sleep()



def main():
    bot = Bot('simple_vel')
    bot.run()
    rospy.loginfo("Bot End")

if __name__ == '__main__':
    main()
