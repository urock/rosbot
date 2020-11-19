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
from geometry_msgs.msg import Pose, PoseStamped
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

        self.cmd_freq = 10 # Hz
        self.rate = rospy.Rate(self.cmd_freq)
        self.tf_listener = tf.TransformListener()

        self.state = (0, 0, 0, 0, 0, 0)    # x, y, yaw, vx, vy,  w

        # self.point_cloud_sub = rospy.Subscriber(point_cloud_topic, PointCloud2, self.PointCloudCallback)

        # self.cmd_pub = rospy.Publisher('/mover/cmd', mover_cmd, queue_size=5)
        


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


    

    def run(self):
        while not rospy.is_shutdown():
            self.get_current_state()
            self.print_state()
            self.rate.sleep()



def main():
    bot = Bot('simple_vel')
    
    bot.run()

    rospy.loginfo("Bot End")

if __name__ == '__main__':
    main()
