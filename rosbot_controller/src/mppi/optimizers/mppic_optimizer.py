from utils.dtypes import State
import numpy as np
import sys
import math

import rospy
sys.path.append("..")


class MPPICOptimizer:
    def __init__(self, obstacles, control_generator, cost, next_control_policy):
        self._get_params()

        self.cost = cost
        self.generator = control_generator
        self.next_control_policy = next_control_policy
        self.temperature: float

        self.obstacles = np.array([np.array(obstacle) for obstacle in obstacles])
        self.reference_trajectory: np.ndarray
        self.reference_intervals: np.array
        self.curr_trajectories: np.ndarray

        self.weights = {}
        self.powers = {}

        self.count_ahead = 10
        self.count_behind = 0

        self.curr_offset = 0

    def _get_params(self):
        self.iter_count = int(rospy.get_param("~optimizer/iter_count", 1))
        self.traj_lookahead = int(rospy.get_param("~optimizer/traj_lookahead", 2))

    def calc_next_control(self, goal_idx):
        start = rospy.Time.now()

        for _ in range(self.iter_count):
            self.count_ahead, self.count_behind = self._calc_considered_ref_points(goal_idx)
            self._optimize(goal_idx)

        self.curr_exec_time = (rospy.Time.now() - start).to_sec()

        self.curr_offset = min(round(self.curr_exec_time / self.generator.dt),
                               self.generator.time_steps - 1)

        control = self.generator.get_control(self.curr_offset)
        self.generator.displace_controls(self.curr_offset)

        rospy.loginfo("Optimizer. Exec Time: {:.4f}, Goal: {} ".format(
            self.curr_exec_time, goal_idx))
        return control

    def get_offset_time(self):
        return self.curr_offset * self.generator.dt

    def _optimize(self, goal_idx: int):
        self.curr_trajectories = self.generator.generate_trajectories()

        beg = max(0, goal_idx - self.count_behind)
        end = min(goal_idx + self.count_ahead, len(self.reference_trajectory) - 1) + 1

        self.reference_considered = self.reference_trajectory[beg:end, :3]
        self.intervals_considered = self.reference_intervals[beg:end - 1]

        costs = self.cost(
            self.curr_trajectories,
            self.reference_considered,
            self.intervals_considered,
            self.obstacles,
            self.weights,
            self.powers
        )

        self.generator.curr_control_seq = self.next_control_policy(
            costs, self.generator.controls_batch, self.temperature)

    def _calc_considered_ref_points(self, goal_idx):
        count_ahead = 0
        dist = 0
        for q in range(goal_idx, len(self.reference_intervals)):
            dist += self.reference_intervals[q]
            count_ahead += 1
            if dist > self.traj_lookahead:
                break

        count_behind = 0
        dist = 0

        for w in range(goal_idx - 1, -1, -1):
            dist += self.reference_intervals[w]
            count_behind += 1
            if dist > self.traj_lookahead:
                break

        return count_ahead, count_behind
