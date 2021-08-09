import math
import numpy as np


class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, w=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.w = w

    def to_numpy(self):
        return np.array([self.x, self.y, self.yaw, self.v, self.w])

    def __repr__(self):
        return "x -> {:.2f}, y -> {:.2f}, yaw -> {:.2f}, v -> {:.2f} w -> {:.2f}".format(
            self.x, self.y, self.yaw, self.v, self.w
        )

    def size(self):
        return 5


class Control:
    def __init__(self, v=0.0, w=0.0):
        self.v = v
        self.w = w

    def to_numpy(self):
        return np.array([self.v, self.w])

    def __repr__(self):
        return "v -> {:.2f}, w -> {:.2f}".format(self.v, self.w)


class Constraints:
    def __init__(self, v_max=0, w_max=0):
        self.v_min = v_max
        self.w_min = w_max

        self.v_max = v_max
        self.w_max = w_max

    def __repr__(self):
        return "v_max -> {:.2f}, w_max -> {:.2f}".format(self.v_max, self.w_max)
