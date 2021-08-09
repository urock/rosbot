from utils.geometry import quaternion_to_euler
from utils.dtypes import State
import rospy
from nav_msgs.msg import Odometry
import numpy as np

import sys
sys.path.append("..")


class Odom:
    """
    Class keep state of the robot, updating it from external source (tf message)
    """

    def __init__(self):
        self.state = State()
        self._odom_topic = rospy.get_param("~robot/odom_topic", "/odom")
        rospy.Subscriber(self._odom_topic, Odometry, self._odometry_cb)

    def _odometry_cb(self, odom):
        self.state.x = odom.pose.pose.position.x
        self.state.y = odom.pose.pose.position.y
        self.state.yaw = quaternion_to_euler(odom.pose.pose.orientation)[0]
        self.state.v = odom.twist.twist.linear.x
        self.state.w = odom.twist.twist.angular.z
