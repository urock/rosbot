#!/usr/bin/env python3

import numpy as np
import rospy


class FiguresGenerator:
    def __init__(self):
        self.generators = {
            'sin': self._make_sin,
            'polygon': self._make_polygon,
        }

    def generate(self, path):
        type = path['type']
        args = path['args']

        rospy.loginfo("Generator: generating path of type: '{}' with args: {}".  format(type, args))
        return self.generators[type](**args)

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
