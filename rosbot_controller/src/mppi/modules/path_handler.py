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
        self.has_path = False
        self.path_come_time: float
        self.path_intervals = np.zeros(shape=(0))

        self._path = np.empty(shape=(0, 3))
        self._path_sub = rospy.Subscriber("/path", Path, self._path_cb)

    @property
    def path(self):
        self.has_path = False
        return self._path

    def _path_cb(self, msg):
        for pose in msg.poses:
            x, y = pose.pose.position.x, pose.pose.position.y
            yaw = quaternion_to_euler(pose.pose.orientation)[0]
            self._path = np.append(self.path, [[x, y, yaw]], axis=0)


        self.path_intervals = np.linalg.norm(self._path[1:, :2] - self._path[:-1, :2], axis=1)


        self.has_path = True
        self.path_come_time = time()
