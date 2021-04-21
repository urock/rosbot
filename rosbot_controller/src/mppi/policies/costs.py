import numpy as np

DESIRED_V_WEIGHT = 3.0
YAW_WEIGHT = 0.45
LAST_GOAL_WEIGHT = 5


K_NEAREST = 3


def nearest_cost(state, ref_traj, traj_lookahead, goal_idx, desired_v, goals_interval):
    """ Cost according to k nearest ref_traj points

    Args:
        state: np.ndarray of shape [batch_size, time_steps, 3] where 3 for x, y, yaw
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

    x_dist = (state[:, :, :1] - ref[:, :1].reshape(-1))**2
    y_dist = (state[:, :, 1:2] - ref[:, 1:2].reshape(-1))**2
    yaw_dist = (state[:, :, 2:3] - ref[:, 2:3].reshape(-1))**2

    dist = x_dist + y_dist + YAW_WEIGHT * yaw_dist

    k = min(K_NEAREST, len(ref_traj) - goal_idx)
    dist = np.partition(dist, k - 1, axis=2)[:, :, :k] * np.arange(1, k + 1)

    costs += dist.sum(2).sum(1)
    return costs


# WARN This is the pre-alpha version. Bugs expected
def triangle_cost(state, ref_traj, traj_lookahead, goal_idx, desired_v, goals_interval):
    v = state[:, :, 3]
    costs = DESIRED_V_WEIGHT * (v - desired_v)**2
    costs = costs.sum(axis=1)

    end = goal_idx + traj_lookahead
    end = min(end, len(ref_traj))

    ref = ref_traj[goal_idx:end, :2]

    # batch_size time_steps x min(traj_lookahead, len(ref_traj))
    x_dist = (state[:, :, :1] - ref[:, :1].reshape(1, 1, -1))**2
    y_dist = (state[:, :, 1:2] - ref[:, 1:2].reshape(1, 1, -1))**2

    dist = np.sqrt(x_dist + y_dist)
    dist_idxs = np.argpartition(dist, 1, axis=2)[:, :, :2]

    b_sides = dist[:, :, dist_idxs[:,0]]
    c_sides = dist[:, :, dist_idxs[:,1]]
    a_sides = dist_L2(ref_traj[dist_idxs[0]], ref_traj[dist_idxs[1]])

    half_perims = (a_sides + b_sides + c_sides) / 2.0
    height = 2.0 / a_sides * np.sqrt(half_perims * (half_perims - a_sides) * (half_perims - b_sides) * (half_perims - c_sides))

    height = height.squeeze(2).sum(1)

    costs += height**2
    return costs

def dist_L2(lhs, rhs):
    return np.sqrt((lhs[0] - rhs[0]) ** 2 + (lhs[0] - rhs[1])**2)
