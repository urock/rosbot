import math
import numpy as np


class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, w=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.w = w

    def to_str(self):
        return "x -> {:.2f}, y -> {:.2f}, yaw -> {:.2f}".format(self.x, self.y, self.yaw)

    def size(self):
        return 5


class Control:
    def __init__(self, v=0.0, w=0.0):
        self.v = v
        self.w = w

    def to_str(self):
        return "v -> {:.2f}, w -> {:.2f}".format(self.v, self.w)


class Constraints:
    def __init__(self, v_max=0, w_max=0):
        self.v_max = v_max
        self.w_max = w_max

    def to_str(self):
        return "v_max -> {:.2f}, w_max -> {:.2f}".format(self.v_max, self.w_max)


def dist_L2(lhs, rhs):
    return np.sqrt((lhs.x - rhs.x) ** 2 + (lhs.y - rhs.y)**2)


def dist_L2_np(lhs, rhs_np):
    return np.sqrt((lhs.x - rhs_np[0]) ** 2 + (lhs.y - rhs_np[1])**2)
