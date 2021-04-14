import numpy as np
from numba import jit
from scipy.spatial import distance

import sys


def sum_loss(trajectories, reference_traj, traj_lookahead, goal_idx):
    loss = np.zeros(shape=(trajectories.shape[0], trajectories.shape[1]))
    x = trajectories[:, :, 0]
    y = trajectories[:, :, 1]
    yaw = trajectories[:, :, 2]

    traj_end = reference_traj.shape[0]
    end = goal_idx + traj_lookahead + 1
    end = min(traj_end, end)

    for q in range(goal_idx, end):
        goal = reference_traj[q]
        loss += ((x - goal[0])**2 + (y-goal[1])**2 + (yaw-goal[2])**2)*(q+1)

    return loss.sum(axis=1)


def order_loss(trajectories, reference_traj, traj_lookahead, goal_idx):
    loss = np.zeros(shape=(trajectories.shape[0], trajectories.shape[1]))
    x = trajectories[:, :, 0]
    y = trajectories[:, :, 1]

    goals = np.copy(trajectories)

    traj_end = reference_traj.shape[0]
    end = goal_idx + traj_lookahead + 1
    min_end = min(traj_end, end)

    steps = min_end - goal_idx
    goals[:, 0:steps, :] = reference_traj[goal_idx: min_end]
    goals[:, steps:, :] = reference_traj[min_end - 1]

    loss += (x - goals[:, :, 0])**2 + (y-goals[:, :, 1])**2

    return loss.sum(axis=1)


def nearest_loss(trajectories, reference_traj, traj_lookahead, goal_idx):
    """ Loss according to nearest reference_traj

    Args:
        trajectories: np.array of shape [batch_size, time_steps, 3] where 3 for x, y, yaw
        reference_traj: np.array of shape [ref_traj_size, 3] where 3 for x, y, yaw
        traj_lookahead: int  
        goal_idx: int

    Return:
        np.array of shape [batch_size]
    """
    i_dim = trajectories.shape[0]
    ii_dim = trajectories.shape[1]
    r_dim = reference_traj.shape[0]
    yaw_weight = 0.5

    traj_end = r_dim
    end = goal_idx + traj_lookahead
    end = min(end, traj_end)

    ref_traj_cropped = reference_traj[goal_idx: end]
    traj_reshaped = np.reshape(trajectories, (i_dim * ii_dim, 3))

    traj_reshaped[:,2:3] = traj_reshaped[:,2:3] * yaw_weight 
    reference_traj[:,2:3] = reference_traj[:,2:3] * yaw_weight 

    distances = distance.cdist(traj_reshaped[:, :3], ref_traj_cropped[:, :3], 'sqeuclidean')
    distances = np.min(distances, axis=1)
    distances = distances.reshape(i_dim, ii_dim)

    return np.sum(distances, axis=1)
