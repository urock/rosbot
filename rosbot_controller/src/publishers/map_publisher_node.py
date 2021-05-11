#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose

from std_srvs.srv import Empty, EmptyResponse

from map_generator import MapGenerator
from map_handler import MapHandler


class MapPublisher:
    def __init__(self):
        self._map_frame = rospy.get_param('~map_frame', '/odom')

        self._map_topic = rospy.get_param('~map_topic', '/map')
        self._map_resolution = rospy.get_param('~map_resolution', 0.1)

        self._model_name = rospy.get_param('~gz_model_name', 'rosbot')
        self._reference_name = rospy.get_param('~gz_reference_frame', 'world')

        self._map_generator = MapGenerator()
        self._map_origin = rospy.get_param('~map_origin_pose', [0, 0])

        origin_pose = Pose()
        origin_pose.position.x = self._map_origin[0]
        origin_pose.position.y = self._map_origin[1]

        self._map_handler = MapHandler(self._map_frame, self._map_topic, origin=origin_pose)

        self._maps = rospy.get_param('~maps', [])
        self._map_idx = 0
        self._next_map = False
        self._next_map_srv = rospy.Service('next_map', Empty, self._next_map_cb)

    def start(self):
        rospy.sleep(2)

        while not rospy.is_shutdown():
            if self._next_map:
                map = self._maps[self._map_idx]
                data = self._map_generator.generate(map)
                self._map_handler.publish(self._map_resolution,
                                          map['args']['width'],
                                          map['args']['height'], data)

                self._map_idx = 0 if self._map_idx == len(self._maps) - 1 else (self._map_idx + 1)
                self._next_map = False

    def _next_map_cb(self, req):
        self._next_map = True
        return EmptyResponse()


def main():
    rospy.init_node("map_pub", anonymous=True)
    node = MapPublisher()
    node.start()

    rospy.spin()


if __name__ == '__main__':
    main()
