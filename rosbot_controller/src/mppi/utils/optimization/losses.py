import numpy as np
from numba import jit


DESIRED_V_WEIGHT = 1
YAW_WEIGHT = 0.45
LAST_GOAL_WEIGHT = 5


@jit(nopython=True)
def sum_loss(state, ref_traj, traj_lookahead, goal_idx, desired_v, goals_interval):
    """ Loss according to nearest ref_traj

    Args:
        state: np.array of shape [batch_size, time_steps, 3] where 3 for x, y, yaw
        ref_traj: np.array of shape [ref_traj_size, 3] where 3 for x, y, yaw
        traj_lookahead: int  
        goal_idx: int

    Return:
        np.array of shape [batch_size]
    """
    loss = np.zeros(shape=(state.shape[0], state.shape[1]))
    x = state[:, :, 0]
    y = state[:, :, 1]
    yaw = state[:, :, 2]
    v = state[:, :, 3]

    traj_end = ref_traj.shape[0]
    end = goal_idx + traj_lookahead
    end = min(traj_end, end)

    for q in range(goal_idx, end):
        goal = ref_traj[q]
        loss += ((x - goal[0])**2 + (y-goal[1])**2 + YAW_WEIGHT * (yaw-goal[2])
                 ** 2) * (LAST_GOAL_WEIGHT if q == (end - 1) else 1)

    loss += DESIRED_V_WEIGHT * (v - desired_v)**2
    return loss.sum(axis=1)


@jit(nopython=True)
def triangle_loss(state, ref_traj, traj_lookahead, goal_idx, desired_v, goals_interval):
    """ Loss according to nearest ref_traj

    Args:
        state: np.array of shape [batch_size, time_steps, 3] where 3 for x, y, yaw
        ref_traj: np.array of shape [ref_traj_size, 3] where 3 for x, y, yaw
        traj_lookahead: int  
        goal_idx: int

    Return:
        np.array of shape [batch_size]
    """
    v = state[:, :, 3]
    loss = DESIRED_V_WEIGHT * (v - desired_v)**2
    loss = loss.sum(axis=1)

    end = goal_idx + traj_lookahead
    end = min(end, len(ref_traj))
    ref_traj_cropped = ref_traj[:, :2]

    batch_size = state.shape[0]
    time_steps = state.shape[1]
    height_distances = np.zeros(shape=(batch_size, time_steps))

    for q in range(len(state)):
        for w in range(len(state[q])):
            curr_pt = state[q][w][:2]
            if goal_idx == 0:
                height_distances[q, w] = get_dist(curr_pt, ref_traj_cropped[0])
                continue

            dist_to_closest, idx_closest = find_closest(curr_pt, ref_traj_cropped, goal_idx, end)
            b_side = dist_to_closest
            c_side = get_dist(curr_pt, ref_traj_cropped[idx_closest - 1])
            a_side = goals_interval
            height_distances[q, w] = get_height(a_side, b_side, c_side)

    loss += height_distances.sum(axis=1)
    return loss


@jit(nopython=True)
def find_closest(lhs, seq, seq_start, seq_end):
    min_dist = 1e10
    min_idx = 0

    for q in range(seq_start, seq_end):
        curr_dist = get_dist(lhs, seq[q])
        if min_dist > curr_dist:
            min_dist = curr_dist
            min_idx = q

    return min_dist, min_idx


@jit(nopython=True)
def get_dist(a, b):
    value = np.linalg.norm(a-b)
    return value


@jit(nopython=True)
def get_height(a, b, c):
    p = (a + b + c)/2.0
    h = 2.0/a * np.sqrt(p * (p - a) * (p - b) * (p - c))
    return h
