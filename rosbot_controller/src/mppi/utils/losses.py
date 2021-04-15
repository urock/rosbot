import numpy as np
from scipy.spatial import distance


def sum_loss(state, reference_traj, traj_lookahead, goal_idx, desired_v):
    loss = np.zeros(shape=(state.shape[0], state.shape[1]))
    x = state[:, :, 0]
    y = state[:, :, 1]
    yaw = state[:, :, 2]
    v = state[:, :, 3]

    yaw_weight = 0.45
    v_desiried_weight = 15

    traj_end = reference_traj.shape[0]
    end = goal_idx + traj_lookahead + 1
    end = min(traj_end, end)

    for q in range(goal_idx, end):
        goal = reference_traj[q]
        loss += ((x - goal[0])**2 + (y-goal[1])**2 + yaw_weight * (yaw-goal[2])**2) * (q+1) 

    loss += v_desiried_weight * (v - desired_v)**2 
    return loss.sum(axis=1)


def order_loss(trajs, reference_traj, traj_lookahead, goal_idx, desired_v):
    loss = np.zeros(shape=(trajs.shape[0], trajs.shape[1]))
    x = trajs[:, :, 0]
    y = trajs[:, :, 1]

    goals = np.copy(trajs)

    traj_end = reference_traj.shape[0]
    end = goal_idx + traj_lookahead + 1
    min_end = min(traj_end, end)

    steps = min_end - goal_idx
    goals[:, 0:steps, :] = reference_traj[goal_idx: min_end]
    goals[:, steps:, :] = reference_traj[min_end - 1]

    loss += (x - goals[:, :, 0])**2 + (y-goals[:, :, 1])**2

    return loss.sum(axis=1)


def nearest_loss(trajs, reference_traj, traj_lookahead, goal_idx, desired_v):
    """ Loss according to nearest reference_traj

    Args:
        trajectories: np.array of shape [batch_size, time_steps, 3] where 3 for x, y, yaw
        reference_traj: np.array of shape [ref_traj_size, 3] where 3 for x, y, yaw
        traj_lookahead: int  
        goal_idx: int

    Return:
        np.array of shape [batch_size]
    """
    i_dim = trajs.shape[0]
    ii_dim = trajs.shape[1]
    r_dim = reference_traj.shape[0]
    yaw_weight = 0.2

    traj_end = r_dim
    end = goal_idx + traj_lookahead
    end = min(end, traj_end)

    ref_traj_cropped = reference_traj[goal_idx: end]
    traj_reshaped = np.reshape(trajs[:,:,:3], (i_dim * ii_dim, 3))

    traj_reshaped[:, 2:3] = traj_reshaped[:, 2:3] * yaw_weight
    reference_traj[:, 2:3] = reference_traj[:, 2:3] * yaw_weight

    distances = distance.cdist(traj_reshaped[:, :3], ref_traj_cropped[:, :3], 'sqeuclidean')
    distances = np.min(distances, axis=1)

    distances = distances.reshape(i_dim, ii_dim)

    return np.sum(distances, axis=1)
