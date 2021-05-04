#!/usr/bin/env python3

from utils.geometry import euler_to_quaternion
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path

import rospy


class PathsHandler:
    def __init__(self, map_frame, publish_topic):
        self.map_frame = map_frame
        self.path_topic = publish_topic
        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=5)

    def publish(self, x, y, yaw):
        path = self._create_path(x, y, yaw)

        rospy.loginfo("Path Handler: Publishing path ...")
        self.path_pub.publish(path)
        rospy.loginfo("Path Handler: Path Published")

    def _create_path(self, x, y, yaw):
        path = self._make_path()
        for i in range(len(x)):
            pose = self._make_pose(path.header, i, x[i], y[i], yaw[i])
            path.poses.append(pose)

        return path

    def _make_path(self):
        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = rospy.Time.now()
        path.header.seq = 0

        return path

    def _make_pose(self, header, seq, x, y, yaw):
        pose = PoseStamped()
        pose.header = header
        pose.header.seq = seq
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0
        pose.pose.orientation = Quaternion(*euler_to_quaternion(yaw, 0, 0))
        return pose
