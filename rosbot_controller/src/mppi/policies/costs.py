from numba import jit
from numba.experimental import jitclass
import numpy as np


from abc import ABC, abstractmethod


class Cost(ABC):
    @abstractmethod
    def __call__(self, state, reference_trajectory, desired_v, goals_interval):
        """Calculate cost 

        Args:
            state: np.ndarray of shape [batch_size, time_steps, 7] where 3 for x, y, yaw, v, w, v_control, w_control
            ref_traj: np.array of shape [ref_traj_size, 3] where 3 for x, y, yaw
            traj_lookahead: int
            goal_idx: int
            goals_interval: float
            desired_v: float

        Return:
            costs: np.array of shape [batch_size]
        """
class TriangleCost():
    def __call__(self, state, reference_trajectory, desired_v, goals_interval):
        """Cost according to nearest segment.

        Args:
            state: np.ndarray of shape [batch_size, time_steps, 8] where 3 for x, y, yaw, v, w, v_control, w_control, dts
            ref_traj: np.array of shape [ref_traj_size, 3] where 3 for x, y, yaw
            traj_lookahead: int
            goal_idx: int
            goals_interval: float
            desired_v: float

        Return:
            costs: np.array of shape [batch_size]
        """
        v = state[:, :, 3]
        w = state[:, :, 5]

        v_control = state[:, :, 5]
        w_control = state[:, :, 6]

        costs = lin_vel_cost(v, desired_v)
        costs += self._triangle_cost_segments(state, reference_trajectory, goals_interval)

        return costs

    def _triangle_cost_segments(self, state, ref, goals_interval):
        x_dists = state[:, :, :1] - ref[:, 0]
        y_dists = state[:, :, 1:2] - ref[:, 1]
        dists = np.sqrt(x_dists ** 2 + y_dists ** 2)
        triangle_costs = np.empty(shape=(dists.shape[0], dists.shape[1], len(ref) - 1))

        if len(ref) == 1:
            return dists.squeeze(2).sum(1)

        for q in range(len(ref) - 1):
            first_sides = dists[:, :, q]
            second_sides = dists[:, :, q + 1]
            opposite_sides = goals_interval
            triangle_costs[:, :, q] = self._triangle_cost_segment(
                opposite_sides, first_sides, second_sides
            )

        return triangle_costs.min(2).sum(1)

    def _triangle_cost_segment(self, opposite_side, first_sides, second_sides):
        costs = np.empty(shape=first_sides.shape)
        first_obtuse_mask = self._is_angle_obtuse(first_sides, second_sides, opposite_side)
        second_obtuse_mask = self._is_angle_obtuse(second_sides, first_sides, opposite_side)
        h_mask = (~first_obtuse_mask) & (~second_obtuse_mask)

        costs[first_obtuse_mask] = second_sides[first_obtuse_mask]
        costs[second_obtuse_mask] = first_sides[second_obtuse_mask]
        costs[h_mask] = self._heron(opposite_side, first_sides[h_mask], second_sides[h_mask])

        return costs

    def _is_angle_obtuse(self, opposite_side, b, c):
        return opposite_side ** 2 > (b ** 2 + c ** 2)

    def _heron(self, opposite_side, b, c):
        p = (opposite_side + b + c) / 2.0
        h = 2.0 / opposite_side * np.sqrt(p * (p - opposite_side) * (p - b) * (p - c))
        return h


def lin_vel_cost(v, desired_v):
    DESIRED_V_WEIGHT = 4.0
    v_costs = DESIRED_V_WEIGHT * ((v - desired_v)**2).sum(1)
    return v_costs
