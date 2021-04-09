#!/usr/bin/env python3

import math
import numpy as np
import rospy
import time

from mpc_types import State, Control, Constraints, dist_L2

class Rosbot:

    def __init__(self):
        self.state = State()
        self.params = Constraints()
        self.eps_w = 0.001
        self.eps_r = 0.001

        self.v = 0.0
        self.w = 0.0
        self.t = 0.0  # current time in seconds

        self.dist_tolerance = 0.2

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

    def goal_reached(self, goal):
        return  dist_L2(self.state, goal) < self.dist_tolerance


    def calculate_contol(self, goal):
        """Calculate control towards goal

        Args:
            goal: target State
        """
        r = dist_L2(goal)

        azim_goal = math.atan2((goal.y - self.state.y), (goal.x - self.state.x))
        alpha = azim_goal - self.state.yaw

        if (abs(alpha) > math.pi):
            alpha -= np.sign(alpha) * 2 * math.pi

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
