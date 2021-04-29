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
        self.control_generator = control_generator
        self.next_control_policy = next_control_policy

        self.reference_trajectory: np.ndarray
        self.reference_intervals: np.array

        self.curr_trajectories: np.ndarray
        self.curr_offset = 0

        self._iter_count = int(rospy.get_param("~mppic/iter_count", 1))
        self._desired_v = rospy.get_param("~mppic/desired_v", 0.5)

        self._trajectory_lookahead = int(rospy.get_param("~mppic/traj_lookahead", 7))

    def calc_next_control(self, goal_idx):
        start = perf_counter()
        for _ in range(self._iter_count):
            self._optimize(goal_idx)

        self.curr_exec_time = perf_counter() - start
        self.curr_offset = min(math.ceil(self.curr_exec_time / self.control_generator.dt), 
                len(self.control_generator.curr_control_seq) - 1)

        control = self.control_generator.get_control(self.curr_offset)
        self.control_generator.displace_controls(self.curr_offset)

        rospy.loginfo_throttle(2, "Offset: {} Goal: {}. Exec Time {:.4f}, Offset Time: {:.4f} .  \
                \nControl [v, w] = [{:.2f} {:.2f}]. \nOdom    [v, w] = [{:.2f} {:.2f}] \n".format(
            self.curr_offset, goal_idx, self.curr_exec_time, self.get_offset_time(), control.v, control.w,
            self.control_generator.state.v, self.control_generator.state.w))

        self.curr_exec_time = perf_counter() - start
        return control

    def get_offset_time(self):
        return self.curr_offset * self.control_generator.dt

    def _optimize(self, goal_idx: int):
        self.curr_trajectories = self.control_generator.generate_trajectories()

        count = self._calc_considered_point_count(goal_idx)

        beg = max(0, goal_idx - count)
        end = min(goal_idx + count, len(self.reference_trajectory) - 1) + 1

        reference_considered = self.reference_trajectory[beg:end, :3]
        intervals_considered = self.reference_intervals[beg:end - 1]

        costs = self.cost(
            self.curr_trajectories,
            reference_considered,
            intervals_considered,
            self._desired_v
        )

        next_control_seq = self.next_control_policy(costs, self.control_generator.controls_batch)
        self.control_generator.curr_control_seq = next_control_seq
        # print(self.control_generator.curr_control_seq.shape)

    def _calc_considered_point_count(self, goal_idx):
        count = 0
        dist = 0
        for q in range(goal_idx, len(self.reference_intervals)):
            dist += self.reference_intervals[q]
            count += 1
            if dist > self._trajectory_lookahead:
                break

        return count



