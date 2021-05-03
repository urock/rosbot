#!/usr/bin/env python3

import rospy

from std_srvs.srv import Empty, EmptyResponse
from figures_generator import FiguresGenerator
from paths_handler import PathsHandler
from gazebo_state import GazeboState


class PathPublisher:
    def __init__(self):
        self._map_frame = rospy.get_param('~map_frame', '/odom')
        self._path_topic = rospy.get_param('~path_topic', '/path')

        self._model_name = rospy.get_param('~gz_model_name', 'rosbot')
        self._reference_name = rospy.get_param('~gz_reference_frame', 'world')

        self._generator = FiguresGenerator()
        self._gazebo_state = GazeboState(self._model_name, self._reference_name)
        self._path_handler = PathsHandler(self._map_frame, self._path_topic)


        self._paths = rospy.get_param('~paths', [])
        self._path_idx = 0
        self._next_path = False
        self._next_path_srv = rospy.Service('next_path', Empty, self._next_path_cb)

        # self._obstacles = rospy.get_param('~obstacles', [])
        # self._obstacle_idx = 0
        # self._next_obstacle = False
        # self._next_obstacle_srv = rospy.Service('next_obstacle', Empty, self._next_obstacle_cb)

    def start(self):
        rospy.sleep(2)

        while not rospy.is_shutdown():
            if self._next_path:
                self._gazebo_state.reset()
                path = self._paths[self._path_idx]
                x, y, yaw = self._generator.generate(path)
                self._path_handler.publish(x, y, yaw)
                self._update_path_idx()

    def _update_path_idx(self):
        if self._path_idx == len(self._paths) - 1:
            self._path_idx = 0
        else:
            self._path_idx += 1

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
