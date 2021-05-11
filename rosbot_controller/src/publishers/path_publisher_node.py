#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose

from std_srvs.srv import Empty, EmptyResponse
from gazebo_state import GazeboState

from path_generator import PathGenerator
from path_handler import PathsHandler


class PathPublisher:
    def __init__(self):
        self._map_frame = rospy.get_param('~map_frame', '/odom')
        self._path_topic = rospy.get_param('~path_topic', '/path')

        self._model_name = rospy.get_param('~gz_model_name', 'rosbot')
        self._reference_name = rospy.get_param('~gz_reference_frame', 'world')

        self._gazebo_state = GazeboState(self._model_name, self._reference_name)
        self._path_generator = PathGenerator()
        self._path_handler = PathsHandler(self._map_frame, self._path_topic)

        self._paths = rospy.get_param('~paths', [])
        self._path_idx = 0
        self._next_path = False
        self._next_path_srv = rospy.Service('next_path', Empty, self._next_path_cb)

    def start(self):
        rospy.sleep(2)

        while not rospy.is_shutdown():
            if self._next_path:
                self._gazebo_state.reset()
                path = self._paths[self._path_idx]
                x, y, yaw = self._path_generator.generate(path)
                self._path_handler.publish(x, y, yaw)
                self._path_idx = 0 if (self._path_idx == (
                    len(self._paths) - 1)) else (self._path_idx + 1)
                self._next_path = False

    def _next_path_cb(self, req):
        self._next_path = True
        return EmptyResponse()


def main():
    rospy.init_node("path_pub", anonymous=True)
    node = PathPublisher()
    node.start()

    rospy.spin()


if __name__ == '__main__':
    main()
