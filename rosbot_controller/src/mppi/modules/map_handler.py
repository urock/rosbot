import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid


class MapHandler:
    def __init__(self):
        self.map = OccupancyGrid()
        self.coord_map: np.ndarray
        self._map_sub = rospy.Subscriber("/map", OccupancyGrid, self._map_cb)

    def _map_cb(self, map):
        rospy.loginfo("MPPI MapHandler: got map")
        self.map = map

        width = self.map.info.width
        height = self.map.info.height

        self.coord_map = np.zeros(shape=(width, height, 3))  # x, y, occupancy_val
        for x_i in range(width):
            for y_i in range(height):
                self.coord_map[x_i, y_i, 0], self.coord_map[x_i,
                                                            y_i, 1] = self._get_coords(x_i, y_i)
                self.coord_map[x_i, y_i, 2] = self.map.data[x_i + y_i * width]

    def _get_coords(self, x_i, y_i):
        x = self.x_coord(x_i)
        y = self.y_coord(y_i)

        return [x, y]

    def x_coord(self, x_i):
        return (x_i * self.map.info.resolution) + \
            self.map.info.origin.position.x + self.map.info.resolution / 2

    def y_coord(self, y_i):
        return (y_i * self.map.info.resolution) + \
            self.map.info.origin.position.y + self.map.info.resolution / 2
