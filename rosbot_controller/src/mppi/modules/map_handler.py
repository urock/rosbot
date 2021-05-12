import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid

class MapHandler:
    def __init__(self):
        self.map = None
        self.coord_map : np.ndarray
        self._map_sub = rospy.Subscriber("/map", OccupancyGrid, self._map_cb)


    def _map_cb(self, msg):
        self.map = msg

        for self.map.info.width:
            for self.map.info.height:
                get_coords


    def get_coords(self, x_i, y_i):
        x = (x_i * self.map.info.resolution) + self.map.info.origin.position.x + self.map.info.resolution / 2;
        y = (y_i * self.map.info.resolution) + self.map.info.origin.position.y + self.map.info.resolution / 2;

        return [x, y]


    def _make_map(self, origin, resolution, width, height, data):
        time = rospy.Time.now()
        grid = OccupancyGrid()
        grid.header.frame_id = self._map_frame
        grid.header.stamp = time
        grid.info.origin = origin
        grid.info.map_load_time = time
        grid.info.resolution = resolution
        grid.info.width = width
        grid.info.height = height
        grid.data = data
