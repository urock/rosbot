#!/usr/bin/env python
# license removed for brevity

import math
import numpy as np
import rospy
import time
from pydantic import BaseModel, Field

class RobotState():
    x: float = 0.0
    y: float = Field(..., description='y coord', gt=0.1)
    yaw: float

    def __init__(self, x=0.0, y=0.0, yaw=0.0, vx=0.0, vy=0.0, w=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw

    @validator('x')
    def check_x(cls, v):
        if type(x) is not float:
            raise ValueError()

        return v

    def to_str(self):
        return "x -> {:.2f}, y -> {:.2f}, yaw -> {:.2f}".format(self.x, self.y, self.yaw)

rs = RobotState(x=0, y=1, yaw=99)

rs2 = RobotState(**{'y': 1, 'yaw': 88})

rs.dict()

rs.json()

rs_orm = ...

rs.from_orm(rs_orm)

x, y, yaw = input()

rs = RobotState(x=x, y=y, yaw=yaw)

class RobotControl:

    def __init__(self, v=0.0, w=0.0):
        self.v = v
        self.w = w

    def to_str(self):
        return "v -> {:.2f}, w -> {:.2f}".format(self.v, self.w)


class Goal:

    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw

    def to_str(self):
        return "x -> {:.2f}, y -> {:.2f}, yaw -> {:.2f}".format(self.x, self.y, self.yaw)


class Params:

    def __init__(self):
        self.v_max = rospy.get_param("/max_v", 5.0)
        self.w_max = rospy.get_param("/max_w", 2.5)
        self.xy_margin_squared = 0.05


class Rosbot:

    def __init__(self):
        self.state = RobotState()
        self.params = Params()
        self.eps_w = 0.001
        self.eps_r = 0.001
        self.v = 0.0
        self.w = 0.0
        self.t = 0.0  # current time in seconds

    def set_state(self, new_state, last_time=[None]):
        curr_time = time.time()

        if last_time[0] is not None:
            dt = curr_time - last_time[0]
            vx = (new_state.x - self.state.x) / dt
            vy = (new_state.y - self.state.y) / dt
            v = math.sqrt(vx ** 2 + vy ** 2)
            alpha = math.atan2(vy, vx)
            self.v = v * math.cos(alpha - new_state.yaw)

        last_time[0] = curr_time
        
        self.state = new_state

    def dist_to_goal_L2(self, goal):
        """
        param goal - Goal Class object
        """
        return (goal.x - self.state.x) ** 2 + (goal.y - self.state.y) ** 2

    def goal_reached(self, goal):
        dist = np.hypot(goal.x - self.state.x, goal.y - self.state.y)

        # print('>>> goal dist: %f -- %s' % (dist, 'REACHED' if dist < 0.2 else 'NOT REACHED'))

        return dist < 0.2

    def calculate_contol(self, goal):
        """
        Given state calculate control towards goal
        """
        r = self.dist_to_goal_L2(goal)

        azim_goal = math.atan2((goal.y - self.state.y), (goal.x - self.state.x))
        alpha = azim_goal - self.state.yaw

        if (abs(alpha) > math.pi):
            alpha -= np.sign(alpha) * 2 * math.pi
        # print(self.params.v_max)
        # print(self.params.w_max)
        v = self.params.v_max * math.tanh(r) * math.cos(alpha)
        if r > self.eps_r:
            w = self.params.w_max * alpha + math.tanh(r) * math.sin(alpha) * math.cos(alpha) / r
        else:
            w = self.params.w_max * alpha
        return RobotControl(v, w)

    def update_state_by_model(self, control_vector, dt):
        """
        Updates robot state assuming that control has been active for dt seconds
        c : control vector of RobotControl type
        dt : time period in seconds
        """
        v = control_vector.v
        w = control_vector.w

        if abs(w) > self.eps_w:
            rho = v / w

            # step 1. Calc new robot position relative to its previous pose
            x_r = rho * math.sin(w * dt)
            y_r = rho * (1 - math.cos(w * dt))

            # step 2. Transfrom this point to map fixed coordinate system taking into account
            # current robot pose
            self.state.x += x_r * math.cos(self.state.yaw) - y_r * math.sin(self.state.yaw)
            self.state.y += x_r * math.sin(self.state.yaw) + y_r * math.cos(self.state.yaw)
            self.state.yaw += w * dt
        else:
            self.state.x += v * dt * math.cos(self.state.yaw)
            self.state.y += v * dt * math.sin(self.state.yaw)

        return self.state

    def update_state_by_nn_model(self, model, control_vector, dt):
        """
        c : control vector of RobotControl type
        dt : time period in seconds
        """
        control_vector.v
        control_vector.w

        model_input = np.array([[self.v, self.w, control_vector.v, control_vector.w, dt]], dtype=np.float32)
        model_output = model(model_input)
        self.v, self.w = float(model_output[0][0]), float(model_output[0][1])
        vel_vector = RobotControl(self.v, self.w)
        new_state = self.update_state_by_model(vel_vector, dt)
        return new_state


def euler_to_quaternion(yaw, pitch, roll):
    """
    Args:
        yaw: yaw angle
        pitch: pitch angle
        roll: roll angle
    Return:
        quaternion [qx, qy, qz, qw]
    """
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(
        pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(
        pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(
        pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(
        pitch / 2) * np.sin(yaw / 2)

    return [qx, qy, qz, qw]
from typing import overload


def quaternion_to_euler(quaternion):
    """
    Args:
        x, y, z, w
    Return:
        Angles [yaw, pitch, roll]
    """
    x,  y,  z,  w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [yaw, pitch, roll]
