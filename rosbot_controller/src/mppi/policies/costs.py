import numpy as np
from numba import njit


def triangle_cost(state, reference_trajectory, reference_intervals, obstacles, weights, powers):
    """Cost according to nearest segment.

    Args:
        state: np.ndarray of shape [batch_size, time_steps, 8] where 3 for x, y, yaw, v, w, v_control, w_control, dts
        ref_traj: np.array of shape [ref_traj_size, 3] where 3 for x, y, yaw
        desired_v: float
        obstacles: list of points [x, y]
        weights: weights for cost components
        powers: powers for cost components

    Return:
        costs: np.array of shape [batch_size]
    """


    costs = np.empty(shape=state.shape[0])
    obstacles_c = obstacles_cost(state, obstacles, eps=0.15,
                                 weight=weights['obstacle'],
                                 power=powers['obstacle'])

    triangle_c = triangle_cost_segments(state, reference_trajectory, reference_intervals,
                                        weight=weights['reference'], 
                                        power=powers['reference'])

    goal_c = goal_cost(state, reference_trajectory[-1],
                       weight=weights['goal'],
                       power=powers['goal'])

    costs += triangle_c + goal_c + obstacles_c
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
    dists_to_segments = np.empty(len(reference_intervals))
    for i in range(dists.shape[0]):
        for j in range(dists.shape[1]):
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
        cost[i] /= dists.shape[1]

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


def obstacles_cost(state, obstacles, eps, weight, power):
    costs = np.zeros(shape=(state.shape[0]))
    if not len(obstacles):
        return costs

    x, y, r = obstacles[:, 0], obstacles[:, 1], obstacles[:, 2]

    x_dists = state[:, :, :1] - x
    y_dists = state[:, :, 1:2] - y
    dists = np.sqrt(x_dists ** 2 + y_dists ** 2) - r
    dists = dists.min(2).min(1)

    dists[dists < 0] = 0

    # Points close to the obstacle
    approx_mask = (dists < eps)
    costs[approx_mask] = ((eps - dists[approx_mask]) * weight) ** power

    return costs


@njit
def goal_cost(state, goal, weight=1, power=1):
    x_dists = state[:, :, 0] - goal[0]
    y_dists = state[:, :, 1] - goal[1]
    dists = np.sqrt(x_dists ** 2 + y_dists ** 2)

    return weight * (dists[:, -1] ** power)
