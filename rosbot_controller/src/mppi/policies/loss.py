import numpy as np

DESIRED_V_WEIGHT = 1
YAW_WEIGHT = 0.45
LAST_GOAL_WEIGHT = 5


K_NEAREST = 3


def nearest_loss(state, ref_traj, traj_lookahead, goal_idx, desired_v, goals_interval):
    """ Loss according to k nearest ref_traj points

    Args:
        state: np.ndarray of shape [batch_size, time_steps, 3] where 3 for x, y, yaw
        ref_traj: np.array of shape [ref_traj_size, 3] where 3 for x, y, yaw
        traj_lookahead: int  
        goal_idx: int
        goals_interval: gloat
        desired_v: float

    Return:
        losses: np.array of shape [batch_size]
    """
    v = state[:, :, 3]
    losses = DESIRED_V_WEIGHT * (v - desired_v)**2
    losses = losses.sum(axis=1)

    end = goal_idx + traj_lookahead
    end = min(end, len(ref_traj))

    ref = ref_traj[goal_idx:end, :3]

    x_dist = (state[:, :, :1] - ref[:, :1].reshape(-1))**2
    y_dist = (state[:, :, 1:2] - ref[:, 1:2].reshape(-1))**2
    yaw_dist = (state[:, :, 2:3] - ref[:, 2:3].reshape(-1))**2

    dist = x_dist + y_dist + YAW_WEIGHT * yaw_dist

    k = min(K_NEAREST, len(ref_traj) - goal_idx)
    dist = np.partition(dist, k - 1, axis=2)[:, :, :k]

    losses += dist.sum(2).sum(1)
    return losses


# WARN This is the pre-alpha version. Bugs expected
def triangle_loss(state, ref_traj, traj_lookahead, goal_idx, desired_v, goals_interval):
    v = state[:, :, 3]
    losses = DESIRED_V_WEIGHT * (v - desired_v)**2
    losses = losses.sum(axis=1)

    end = goal_idx + traj_lookahead
    end = min(end, len(ref_traj))

    ref = ref_traj[goal_idx:end, :2]

    # batch_size time_steps x min(traj_lookahead, len(ref_traj))
    x_dist = (state[:, :, :1] - ref[:, :1].reshape(1, 1, -1))**2
    y_dist = (state[:, :, 1:2] - ref[:, 1:2].reshape(1, 1, -1))**2

    dist = np.sqrt(x_dist + y_dist)
    dist = np.partition(dist, 1, axis=2)[:, :, :2]

    a_side = goals_interval
    b_side = dist[:, :, :1]
    c_side = dist[:, :, 1:2]

    p = (a_side + b_side + c_side) / 2.0
    h = 2.0 / a_side * np.sqrt(p * (p - a_side) * (p - b_side) * (p - c_side))

    h = h.squeeze(2).sum(1)

    losses += h**2
    return losses
