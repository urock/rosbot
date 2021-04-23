import numpy as np
import rospy

DESIRED_V_WEIGHT = 1.0
YAW_WEIGHT = 0.45
LAST_GOAL_WEIGHT = 5


K_NEAREST = 3


def nearest_cost(state, ref_traj, traj_lookahead, goal_idx, desired_v, goals_interval):
    """ Cost according to k nearest ref_traj points

    Args:
        state: np.ndarray of shape [batch_size, time_steps, 3] where 5 for x, y, yaw, v, w
        ref_traj: np.array of shape [ref_traj_size, 3] where 3 for x, y, yaw
        traj_lookahead: int  
        goal_idx: int
        goals_interval: gloat
        desired_v: float

    Return:
        costs: np.array of shape [batch_size]
    """
    v = state[:, :, 3]
    costs = DESIRED_V_WEIGHT * (v - desired_v)**2
    costs = costs.sum(axis=1)

    end = goal_idx + traj_lookahead
    end = min(end, len(ref_traj))

    ref = ref_traj[goal_idx:end, :3]

    x_dists = state[:, :, :1] - ref[:, 0]
    y_dists = state[:, :, 1:2] - ref[:, 1]
    yaw_dists = state[:, :, 2:3] - ref[:, 2]

    dists = x_dists**2 + y_dists**2 + (YAW_WEIGHT * yaw_dists**2)

    k = min(K_NEAREST, len(ref_traj) - goal_idx)
    dists = np.partition(dists, k - 1, axis=2)[:, :, :k] * np.arange(1, k + 1)

    costs += dists.sum(2).sum(1)
    return costs


# WARN This is the pre-alpha version. Bugs expected
def triangle_cost(state, ref_traj, traj_lookahead, goal_idx, desired_v, goals_interval):
    """ Cost according to nearest segment 

    Args:
        state: np.ndarray of shape [batch_size, time_steps, 5] where 3 for x, y, yaw, v, w
        ref_traj: np.array of shape [ref_traj_size, 3] where 3 for x, y, yaw
        traj_lookahead: int  
        goal_idx: int
        goals_interval: gloat
        desired_v: float

    Return:
        costs: np.array of shape [batch_size]
    """

    v = state[:, :, 3]
    v_costs = DESIRED_V_WEIGHT * (v - desired_v).mean(1)**2
    costs = v_costs

    beg = goal_idx - 1 if goal_idx != 0 else 0
    end = goal_idx + traj_lookahead
    end = min(end, len(ref_traj))

    ref = ref_traj[beg:end, :3]

    x_dists = state[:, :, :1] - ref[:, 0]
    y_dists = state[:, :, 1:2] - ref[:, 1]
    yaw_dists = state[:, :, 2:3] - ref[:, 2]
    dists = np.sqrt(x_dists**2 + y_dists**2)

    yaw_cost = yaw_dists**2
    costs += YAW_WEIGHT * yaw_cost.sum(2).sum(1)

    if len(ref) == 1:
        return (costs + dists.squeeze(2))

    triangle_costs = np.empty(shape=(state.shape[0], state.shape[1], len(ref) - 1))
    for q in range(len(ref) - 1):
        first_sides = dists[:, :, q]
        second_sides = dists[:, :, q + 1]
        opposite_sides = goals_interval

        triangle_costs[:, :, q] = triagle_cost_sides(opposite_sides, first_sides, second_sides)

    costs += triangle_costs.min(2).sum(1)
    return costs


def triagle_cost_sides(opposite_side, first_sides, second_sides):
    costs = np.empty(shape=first_sides.shape)

    first_obtuse_mask = is_angle_obtuse(first_sides, second_sides, opposite_side)
    second_obtuse_mask = is_angle_obtuse(second_sides, first_sides, opposite_side)
    h_mask = (~first_obtuse_mask) & (~second_obtuse_mask)

    costs[first_obtuse_mask] = second_sides[first_obtuse_mask]
    costs[second_obtuse_mask] = first_sides[second_obtuse_mask]
    costs[h_mask] = heron(opposite_side, first_sides[h_mask], second_sides[h_mask])

    return costs


def is_angle_obtuse(opposite_side, b, c):
    return opposite_side**2 > (b**2 + c**2)


def heron(opposite_side, b, c):
    p = (opposite_side + b + c) / 2.0
    h = 2.0 / opposite_side * np.sqrt(p * (p - opposite_side) * (p - b) * (p - c))
    return h
