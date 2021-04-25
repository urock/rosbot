from utils.dtypes import State

import rospy
from time import perf_counter
import numpy as np
import sys

sys.path.append("..")


class MPPICOptimizer():
    def __init__(self, control_generator, cost, next_control_policy):
        self.control_generator = control_generator
        self.cost = cost
        self.next_control_policy = next_control_policy

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
        t = perf_counter() - start

        control = self.control_generator.get_control(0)

        rospy.loginfo_throttle(2, "Goal: {}. Exec Time {:.4f}.  \nControl [v, w] = [{:.2f} {:.2f}]. \nOdom    [v, w] = [{:.2f} {:.2f}] \n".format(
            goal_idx, t, control.v, control.w, self.control_generator.state.v, self.control_generator.state.w))

        return control

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

        next_control_seq = self.next_control_policy(
            costs, self.control_generator.get_controls_batch())
        self.control_generator.set_control_seq(next_control_seq)
