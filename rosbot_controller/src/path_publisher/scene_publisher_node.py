#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose

from std_srvs.srv import Empty, EmptyResponse
from gazebo_state import GazeboState

from path_generator import PathGenerator
from paths_handler import PathsHandler

from map_generator import MapGenerator
from map_handler import MapHandler


class ScenePublisher:
    def __init__(self):
        self._map_frame = rospy.get_param('~map_frame', '/odom')
        self._path_topic = rospy.get_param('~path_topic', '/path')

        self._map_topic = rospy.get_param('~map_topic', '/map')
        self._map_resolution = rospy.get_param('~map_resolution', 0.1)

        self._model_name = rospy.get_param('~gz_model_name', 'rosbot')
        self._reference_name = rospy.get_param('~gz_reference_frame', 'world')

        self._path_generator = PathGenerator()
        self._path_handler = PathsHandler(self._map_frame, self._path_topic)

        self._map_generator = MapGenerator()
        self._map_origin = rospy.get_param('~map_origin_pose', [0, 0])

        origin_pose = Pose()
        origin_pose.position.x = self._map_origin[0]
        origin_pose.position.y = self._map_origin[0]

        self._map_handler = MapHandler(self._map_frame, self._map_topic, origin=origin_pose)

        self._gazebo_state = GazeboState(self._model_name, self._reference_name)

        self._paths = rospy.get_param('~paths', [])
        self._path_idx = 0
        self._next_path = False
        self._next_path_srv = rospy.Service('next_path', Empty, self._next_path_cb)

        self._maps = rospy.get_param('~maps', [])
        self._map_idx = 0
        self._next_map = False
        self._next_map_srv = rospy.Service('next_map', Empty, self._next_map_cb)

    def start(self):
        rospy.sleep(2)

        while not rospy.is_shutdown():
            if self._next_path:
                self._gazebo_state.reset()
                path = self._paths[self._path_idx]
                x, y, yaw = self._path_generator.generate(path)
                self._path_handler.publish(x, y, yaw)
                self._path_idx = self._update_idx(self._path_idx, len(self._paths))
                self._next_path = False

            if self._next_map:
                map = self._maps[self._map_idx]
                data = self._map_generator.generate(map)
                self._map_handler.publish(self._map_resolution,
                                          map['args']['width'], map['args']['height'], data)

                self._map_idx = self._update_idx(self._map_idx, len(self._maps))
                self._next_map = False

    def _update_idx(self, idx, length):
        if idx == length - 1:
            idx = 0
        else:
            idx += 1

        return idx

    def _next_path_cb(self, req):
        self._next_path = True
        return EmptyResponse()

    def _next_map_cb(self, req):
        self._next_map = True
        return EmptyResponse()


def main():
    rospy.init_node("path_pub", anonymous=True)
    node = ScenePublisher()
    node.start()

    rospy.spin()


if __name__ == '__main__':
    main()
