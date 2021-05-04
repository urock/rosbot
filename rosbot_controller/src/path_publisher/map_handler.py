#!/usr/bin/env python3

from nav_msgs.msg import OccupancyGrid
import rospy


class MapHandler:
    def __init__(self, map_frame, map_topic, origin):
        self._map_frame = map_frame
        self._origin = origin
        self._maps_pub = rospy.Publisher(map_topic, OccupancyGrid, queue_size=5)

    def publish(self, resolution, width, height, data):
        map = self._make_map(resolution, width, height, data)

        rospy.loginfo("Map Handler: Publishing Map ...")
        self._maps_pub.publish(map)
        rospy.loginfo("Map Handler: Map Published")

    def _make_map(self, resolution, width, height, data):
        time = rospy.Time.now()
        grid = OccupancyGrid()
        grid.header.frame_id = self._map_frame
        grid.header.stamp = time
        grid.info.origin = self._origin
        grid.info.map_load_time = time
        grid.info.resolution = resolution
        grid.info.width = width
        grid.info.height = height
        grid.data = data

        return grid
