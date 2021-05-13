from utils.dtypes import State
from time import perf_counter
import numpy as np
import sys
import math

import rospy
sys.path.append("..")


class MPPICOptimizer():
    def __init__(self, control_generator, cost, next_control_policy):
        self.cost = cost
        self.generator = control_generator
        self.next_control_policy = next_control_policy

        self._iter_count = int(rospy.get_param("~mppic/iter_count", 1))
        self._desired_v = rospy.get_param("~mppic/desired_v", 0.5)
        self._trajectory_lookahead = int(rospy.get_param("~mppic/traj_lookahead", 2))

        self.reference_trajectory: np.ndarray
        self.reference_intervals: np.array
        self.curr_trajectories: np.ndarray
        self.curr_offset = 0

    def calc_next_control(self, goal_idx):
        start = perf_counter()
        for _ in range(self._iter_count):
            count_ahead, count_behind = self._calc_considered_ref_points(goal_idx)
            self._optimize(goal_idx, count_ahead, count_behind)

        self.curr_exec_time = perf_counter() - start
        self.curr_offset = min(math.ceil(self.curr_exec_time / self.generator.dt),
                               len(self.generator.curr_control_seq) - 1)

        control = self.generator.get_control(self.curr_offset)
        self.generator.displace_controls(self.curr_offset)

        rospy.loginfo_throttle(2, "Offset: {} Goal: {}. Exec Time {:.4f},\
                Offset Time: {:.4f}    \n Control [v, w] = [{:.2f} {:.2f}]. \n Odom    [v, w] = [{:.2f} {:.2f}] \n".format(
            self.curr_offset, goal_idx, self.curr_exec_time, self.get_offset_time(),
            control.v, control.w, self.generator.state.v, self.generator.state.w))

        self.curr_exec_time = perf_counter() - start
        return control

    def get_offset_time(self):
        return self.curr_offset * self.generator.dt

    def _optimize(self, goal_idx: int, ref_count_ahead: int, ref_count_behind: int):
        self.curr_trajectories = self.generator.generate_trajectories()

        beg = max(0, goal_idx - ref_count_behind)
        end = min(goal_idx + ref_count_ahead, len(self.reference_trajectory) - 1) + 1

        self.reference_considered = self.reference_trajectory[beg:end, :3]
        self.intervals_considered = self.reference_intervals[beg:end - 1]

        costs = self.cost(
            self.curr_trajectories,
            self.reference_considered,
            self.intervals_considered,
            self._desired_v
        )

        next_control_seq = self.next_control_policy(costs, self.generator.controls_batch)
        self.generator.curr_control_seq = next_control_seq

    def _calc_considered_ref_points(self, goal_idx):
        count_ahead = 0
        dist = 0
        for q in range(goal_idx, len(self.reference_intervals)):
            dist += self.reference_intervals[q]
            count_ahead += 1
            if dist > self._trajectory_lookahead:
                break

        count_behind = 0
        dist = 0

        for w in range(goal_idx - 1, -1, -1):
            dist += self.reference_intervals[w]
            count_behind += 1
            if dist > self._trajectory_lookahead:
                break

        return count_ahead, count_behind
