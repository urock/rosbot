import rospy
from time import time
import numpy as np

from utils.geometry import quaternion_to_euler
from nav_msgs.msg import Path

class TrajectoryHandler():
    """Class for keeping and managing incoming trajectories"""

    def __init__(self):
        self.trajectory = np.empty(shape=(0, 3))
        self.trajectory_come_time: float

        self._trajectory_sub = rospy.Subscriber("/path", Path, self._trajectory_cb)

    def _trajectory_cb(self, msg):
        for pose in msg.poses:
            x, y = pose.pose.position.x, pose.pose.position.y
            yaw = quaternion_to_euler(pose.pose.orientation)[0]
            self.trajectory = np.append(self.trajectory, [[x, y, yaw]], axis=0)

        self.trajectory_come_time = time()
