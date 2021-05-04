#!/usr/bin/env python3

import rospy


class MapGenerator:
    def __init__(self):
        self.generators = {
            'square_obstacle': self._make_squared_obstacle,
        }

    def generate(self, map):
        type = map['type']
        args = map['args']

        rospy.loginfo(
            "Map Generator: generating '{}' with args: {}".  format(type, args))
        return self.generators[type](**args)

    def _make_squared_obstacle(self, width, height, obstacle={'x_bounds': [1, 2], 'y_bounds': [1, 2]}):
        data = []
        for i in range(height):
            for j in range(width):
                data.append(100 if in_obstacle([i, j], obstacle) else 0)
        return data


def in_obstacle(point, obstacle):
    if point[0] >= obstacle['x_bounds'][0] and point[0] <= obstacle['x_bounds'][1] and \
       point[1] >= obstacle['y_bounds'][0] and point[1] <= obstacle['y_bounds'][1]:
        return True
    else:
        return False
