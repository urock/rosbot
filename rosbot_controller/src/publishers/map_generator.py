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

    def _make_squared_obstacle(self, width, height, obstacles):
        data = []
        for y in range(height):
            for x in range(width):
                data.append(100 if in_obstacles([x, y], obstacles) else 0)
        return data


def in_obstacles(point, obstacles):
    x, y = point
    for obstacle in obstacles:
        if (x >= obstacle['x']) and x < (obstacle['x'] + obstacle['x_size']) and \
           (y >= obstacle['y']) and y < (obstacle['y'] + obstacle['y_size']):
            print(x, y)
            return True

    return False
