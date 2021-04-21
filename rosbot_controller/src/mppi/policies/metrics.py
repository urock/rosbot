import sys
sys.path.append("..")

import numpy as np
from utils.dtypes import dist_L2_np

def mean_dist_metric(ref_traj, path_points, goals_interval):
    error = 0
    ref_count = len(ref_traj)
    for ref in ref_traj:
        min_dist = __find_min_dist(ref, path_points)
        error += min_dist

    return error / ref_count 

def __find_min_dist(ref_pt, path_points):
    min_dist = np.inf
    for pt in path_points:
        curr_dist = dist_L2_np(pt, ref_pt)
        if min_dist >= curr_dist:
            min_dist = curr_dist

    return min_dist
