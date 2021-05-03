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
        path = self._generate(x, y, yaw)

        rospy.loginfo("Path Handler: Publishing path ...")
        self.path_pub.publish(path)
        rospy.loginfo("Path Handler: Path Published")

    def _generate(self, x, y, yaw):
        path_msg = self._create_msg()
        for i in range(len(x)):
            pose = self._create_pose(path_msg.header, i, x[i], y[i], yaw[i])
            path_msg.poses.append(pose)

        return path_msg

    def _create_msg(self):
        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = rospy.Time.now()
        path.header.seq = 0

        return path

    def _create_pose(self, header, seq, x, y, yaw):
        ps = PoseStamped()
        ps.header = header
        ps.header.seq = seq
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = 0
        ps.pose.orientation = Quaternion(*euler_to_quaternion(yaw, 0, 0))
        return ps
