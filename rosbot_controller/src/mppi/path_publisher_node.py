#!/usr/bin/env python3

from copy import copy
import os
import numpy as np

from utils.geometry import euler_to_quaternion
from geometry_msgs.msg import PoseStamped, Quaternion
from std_srvs.srv import Empty, EmptyResponse
from nav_msgs.msg import Path

import rospy

from geometry_msgs.msg import Twist
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState


class PathPublisher:
    def __init__(self):
        self.map_frame = rospy.get_param('~map_frame', '/odom')
        self.path_topic = rospy.get_param('~path_topic', '/path')

        self._paths = rospy.get_param(
            '~paths', [
                {
                    'type': 'sin',
                    'args': {'step': 0.1, 'amplitude': 1.1, 'freq': 1.2}
                }
            ]
        )

        self.path_finished = False
        self.finishd_srv_name = 'path_finished'
        self.path_finished_srv = rospy.Service(self.finishd_srv_name, Empty, self.path_finished_cb)
        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=5)

        self.generators = {
            'sin': self._make_sin,
            'polygon': self._make_polygon,
        }

        self.set_state_proxy = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.default_state = ModelState()
        self.default_state.model_name = rospy.get_param('~gz_model_name', 'rosbot')
        self.default_state.reference_frame = rospy.get_param('~gz_reference_frame', 'world')

    def path_finished_cb(self, req):
        self.path_finished = True
        return EmptyResponse()

    def start(self):
        rospy.sleep(2)

        while not rospy.is_shutdown():
            for paths in self._paths:
                self._reset_gz_robot_state()
                path_msg = self.generate_path(paths)
                rospy.loginfo("Publishing generated path ...")
                self.path_pub.publish(path_msg)
                rospy.loginfo("Path published")

                while not self.path_finished:
                    rospy.sleep(1)

                self.path_finished = False

    def generate_path(self, path):
        path_msg = self._create_path_msg()

        rospy.loginfo("Generating path of type: '{}' with args: {}".
                      format(path['type'], path['args']))

        x, y, yaw = self.generators[path['type']](**path['args'])

        for i in range(len(x)):
            pose = self._create_pose(path_msg.header, i, x[i], y[i], yaw[i])
            path_msg.poses.append(pose)

        return path_msg

    def _create_path_msg(self):
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

    def _make_sin(self, step=0.1, amplitude=1.0, freq=1.0, reverse=False):
        K = -1 if reverse == True else 1

        x = np.arange(0, 2*np.pi, 0.01, dtype=float)
        y = amplitude * np.sin(freq * x)

        dist = np.sqrt((x[1:] - x[:-1])**2 + (y[1:] - y[:-1])**2).sum()
        step_count = int(dist / step)

        x = np.linspace(0, 2*np.pi, step_count, dtype=float)
        y = amplitude * np.sin(freq * x)
        yaw = np.arctan(amplitude * freq * np.cos(freq * x))

        return x, y, yaw

    def _make_polygon(self, step=0.1, edges=[[0, 0], [0, 1], [1, 1], [1, 0]]):
        x = []
        y = []
        yaw = []

        edges.append(edges[0])

        for i in range(len(edges) - 1):
            x_1 = edges[i][0]
            x_2 = edges[i + 1][0]
            y_1 = edges[i][1]
            y_2 = edges[i + 1][1]

            dist_x = x_2 - x_1
            dist_y = y_2 - y_1
            dist = np.sqrt((dist_x**2 + dist_y**2))
            step_num = int(dist / step)

            x_curr = np.linspace(x_1, x_2, num=step_num).tolist()
            y_curr = np.linspace(y_1, y_2, num=step_num).tolist()
            yaw_curr = np.zeros(step_num).tolist()

            x.extend(x_curr)
            y.extend(y_curr)
            yaw.extend(yaw_curr)

        return x, y, yaw

    def _reset_gz_robot_state(self):
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_state_proxy(self.default_state)


def main():
    rospy.init_node("path_pub", anonymous=True)
    node = PathPublisher()
    node.start()

    rospy.spin()


if __name__ == '__main__':
    main()
