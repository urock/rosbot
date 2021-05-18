import numpy as np
from numba import njit


# def nearest_cost(state, reference_trajectory, reference_intervals, obstacles, desired_v):
#     """Cost according to k nearest points.

#     Args:
#         state: np.ndarray of shape [batch_size, time_steps, 8]
#                where 3 for x, y, yaw, v, w, v_control, w_control, dts
#         ref_traj: np.array of shape [ref_traj_size, 3] where 3 for x, y, yaw
#         traj_lookahead: int
#         goal_idx: int
#         goals_interval: float
#         desired_v: float

#     Return:
#         costs: np.array of shape [batch_size]
#     """
#     v = state[:, :, 3]

#     costs = lin_vel_cost(v, desired_v)
#     costs += nearest_cost_for_k_ellements(state, reference_trajectory, 3)
#     costs += obstacles_cost(state, obstacles)
#     costs += goal_cost(state, reference_trajectory[-1])

#     return costs


# def nearest_cost_for_k_ellements(state, reference_trajectory, k_idx):
#     x_dists = state[:, :, :1] - reference_trajectory[:, 0]
#     y_dists = state[:, :, 1:2] - reference_trajectory[:, 1]
#     dists = x_dists ** 2 + y_dists ** 2

#     k = min(k_idx, len(reference_trajectory))
#     dists = np.partition(dists, k - 1, axis=2)[:, :, :k] * np.arange(1, k + 1)

#     return dists.min(2).sum(1)


def triangle_cost(state, reference_trajectory, reference_intervals, obstacles, desired_v):
    """Cost according to nearest segment.

    Args:
        state: np.ndarray of shape [batch_size, time_steps, 8] where 3 for x, y, yaw, v, w, v_control, w_control, dts
        ref_traj: np.array of shape [ref_traj_size, 3] where 3 for x, y, yaw
        desired_v: float

    Return:
        costs: np.array of shape [batch_size]
    """

    costs = np.empty(shape=state.shape[0])

    # v = state[:, :, 3]
    # lin_vel_c = lin_vel_cost(v, desired_v, 15)

    obstacles_c = obstacles_cost(state, obstacles, limit=0.0, eps=0.10, inside_penalty=1000,
                                 outside_slope_weight=2, outside_power=1)

    triangle_c = triangle_cost_segments(state, reference_trajectory, reference_intervals,
                                        weight=1, power=1)

    goal_c = goal_cost(state, reference_trajectory[-1],
                       weight=75, power=1)

    costs += obstacles_c + triangle_c + goal_c
    return costs


@njit
def triangle_cost_segments(state, reference_trajectory, reference_intervals, weight=1, power=1):

    x_dists = state[:, :, :1] - reference_trajectory[:, 0]
    y_dists = state[:, :, 1:2] - reference_trajectory[:, 1]
    dists = np.sqrt(x_dists ** 2 + y_dists ** 2)

    if len(reference_trajectory) == 1:
        return dists.sum(2).sum(1)

    first_sides = dists[:, :, :-1]
    second_sides = dists[:, :, 1:]
    opposite_sides = reference_intervals

    first_obtuse_mask = is_angle_obtuse(first_sides, second_sides, opposite_sides)
    second_obtuse_mask = is_angle_obtuse(second_sides, first_sides, opposite_sides)

    cost = np.zeros(shape=(dists.shape[0]))
    for i in range(dists.shape[0]):
        for j in range(dists.shape[1]):
            dists_to_segments = np.empty(len(reference_intervals))
            for k in range(len(reference_intervals)):
                first_side = first_sides[i, j, k]
                second_side = second_sides[i, j, k]
                opposite_side = opposite_sides[k]
                if is_angle_obtuse(first_side, second_side, opposite_side):
                    dists_to_segments[k] = first_side
                elif is_angle_obtuse(second_side, first_side, opposite_side):
                    dists_to_segments[k] = second_side
                else:
                    dists_to_segments[k] = heron(
                        opposite_side,
                        first_side,
                        second_side
                    )
            cost[i] += dists_to_segments.min()

    return (cost*weight)**power


@njit
def is_angle_obtuse(opposite_side, b, c):
    return opposite_side ** 2 > (b ** 2 + c ** 2)


@njit
def heron(opposite_side, b, c):
    eps = 0.000001
    if abs(opposite_side) < eps:
        return min(b, c)

    p = (opposite_side + b + c) / 2.0
    h = 2.0 / opposite_side * np.sqrt(p * (p - opposite_side) * (p - b) * (p - c))
    return h


def obstacles_cost(state, obstacles, limit, eps, inside_penalty, outside_slope_weight, outside_power):
    costs = np.zeros(shape=(state.shape[0]))

    if not len(obstacles):
        return costs

    x, y, r = obstacles[:, 0], obstacles[:, 1], obstacles[:, 2]

    # Distance to the obstacle, including robot's size
    r = r + limit
    x_dists = state[:, :, :1] - x
    y_dists = state[:, :, 1:2] - y
    dists = np.sqrt(x_dists ** 2 + y_dists ** 2) - r
    dists = dists.min(2).min(1)

    # Points inside the obstacle
    costs[dists <= 0] = inside_penalty

    # Points close to the obstacle
    approx_mask = (dists > 0) & (dists < eps)
    costs[approx_mask] = ((eps - dists[approx_mask]) * outside_slope_weight) ** outside_power

    return costs


def goal_cost(state, goal, weight=1, power=1):
    x_dists = state[:, :, 0] - goal[0]
    y_dists = state[:, :, 1] - goal[1]
    dists = np.sqrt(x_dists ** 2 + y_dists ** 2)

    return weight * (dists[:, -1] ** power)


# @njit
def lin_vel_cost(v, desired_v, weight = 2):
    v_costs = weight * ((v - desired_v)**2).mean(1)
    return v_costs
