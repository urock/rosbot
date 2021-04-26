from utils.geometry import quaternion_to_euler
import rospy
from time import time
import numpy as np
from nav_msgs.msg import Path

import sys
sys.path.append("..")


class PathHandler():
    """Class for keeping and managing incoming trajectories
    """

    def __init__(self):
        self.path = np.empty(shape=(0, 3))
        self.path_come_time: float
        self.has_path = False

        self._path_sub = rospy.Subscriber("/path", Path, self._path_cb)

    def get_path(self):
        self.has_path = False
        return self.path

    def _path_cb(self, msg):
        for pose in msg.poses:
            x, y = pose.pose.position.x, pose.pose.position.y
            yaw = quaternion_to_euler(pose.pose.orientation)[0]
            self.path = np.append(self.path, [[x, y, yaw]], axis=0)

        self.has_path = True
        self.path_come_time = time()
