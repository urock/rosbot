from utils.geometry import quaternion_to_euler
from utils.dtypes import State
import rospy
from nav_msgs.msg import Odometry
import numpy as np

import sys
sys.path.append("..")


class Odom:
    """
    Class keeps state of the robot, updating it from external source (tf message)
    """

    def __init__(self):
        self.map_frame = rospy.get_param("~robot/map_frame", "odom")
        self.base_frame = rospy.get_param("~robot/base_frame", "base_link")

        self.curr_state = State()

        rospy.Subscriber('odom', Odometry, self._odometry_cb)

    def _odometry_cb(self, odom):
        self.curr_state.x = odom.pose.pose.position.x
        self.curr_state.y = odom.pose.pose.position.y
        self.curr_state.yaw = quaternion_to_euler(odom.pose.pose.orientation)[0]
        self.curr_state.v = odom.twist.twist.linear.x
        self.curr_state.w = odom.twist.twist.angular.z
