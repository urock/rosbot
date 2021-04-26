from utils.dtypes import State
from time import perf_counter
import numpy as np
import sys
import math

import rospy
sys.path.append("..")


class MPPICOptimizer():
    def __init__(self, control_generator, cost, next_control_policy):
        self.control_generator = control_generator
        self.cost = cost
        self.next_control_policy = next_control_policy
        self.curr_offset = 0

        self._iter_count = int(rospy.get_param("~mppic/iter_count", 1))
        self._desired_v = rospy.get_param("~mppic/desired_v", 0.5)
        self._trajectory_lookahead = int(rospy.get_param("~mppic/traj_lookahead", 7))
        self._goals_interval = rospy.get_param("~mppic/goals_interval", 0.1)

        self._reference_trajectory: np.ndarray
        self._curr_trajectories: np.ndarray

    def get_curr_trajectories(self):
        return self._curr_trajectories

    def set_reference_trajectory(self, trajectory):
        self._reference_trajectory = trajectory

    def set_state(self, state):
        self.control_generator.state = state

    def calc_next_control(self, goal_idx):
        start = perf_counter()
        for _ in range(self._iter_count):
            self._optimize(goal_idx)

        self.curr_exec_time = perf_counter() - start
        self.curr_offset = math.ceil(self.curr_exec_time / self.control_generator.dt)

        control = self.control_generator.get_control(self.curr_offset)
        self.control_generator.displace_controls(self.curr_offset)

        rospy.loginfo_throttle(2, "Offset: {} Goal: {}. Exec Time {:.4f}, Offset Time: {:.4f} .  \
                \nControl [v, w] = [{:.2f} {:.2f}]. \nOdom    [v, w] = [{:.2f} {:.2f}] \n".format(

            self.curr_offset, goal_idx, self.curr_exec_time, self.get_offset_time(), control.v, control.w, self.control_generator.state.v, self.control_generator.state.w))

        self.curr_exec_time = perf_counter() - start
        return control

    def get_offset_time(self):
        return self.curr_offset * self.control_generator.dt

    def _optimize(self, goal_idx: int):
        self._curr_trajectories = self.control_generator.generate_trajectories()

        costs = self.cost(
            self._curr_trajectories,
            self._reference_trajectory,
            self._trajectory_lookahead,
            goal_idx,
            self._desired_v,
            self._goals_interval,
        )

        next_control_seq = self.next_control_policy(costs,
                                                    self.control_generator.get_controls_batch())
        self.control_generator.set_control_seq(next_control_seq)
