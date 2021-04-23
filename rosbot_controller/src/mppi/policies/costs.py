import numpy as np
import rospy


class NearestCost:
    def __init__(self, k_nearest):
        self.k_nearest = k_nearest

    def __call__(self, state, ref_traj, traj_lookahead, goal_idx, desired_v, goals_interval):
        """ Cost according to k nearest ref_traj points

        Args:
            state: np.ndarray of shape [batch_size, time_steps, 3] where 5 for x, y, yaw, v, w
            ref_traj: np.array of shape [ref_traj_size, 3] where 3 for x, y, yaw
            traj_lookahead: int  
            goal_idx: int
            goals_interval: float
            desired_v: float

        Return:
            costs: np.array of shape [batch_size]
        """


        v = state[:, :, 3]

        beg = goal_idx - 1 if goal_idx != 0 else 0
        end = goal_idx + traj_lookahead
        end = min(end, len(ref_traj))
        ref = ref_traj[beg:end, :3]

        costs = lin_vel_cost(v, desired_v)
        costs += ang_cost(state[:,:, 2:3], ref[:, 2])
        costs += self.nearest_cost_k_ellements(state, ref)

        return costs

    def nearest_cost_k_ellements(self, state, ref):
        x_dists = state[:, :, :1] - ref[:, 0]
        y_dists = state[:, :, 1:2] - ref[:, 1]
        dists = x_dists**2 + y_dists**2 

        k = min(self.k_nearest, len(ref))
        dists = np.partition(dists, k - 1, axis=2)[:, :, :k] * np.arange(1, k + 1)

        return dists.sum(2).sum(1)


class TriangleCost:
    def __call__(self, state, ref_traj, traj_lookahead, goal_idx, desired_v, goals_interval):
        """ Cost according to nearest segment 

        Args:
            state: np.ndarray of shape [batch_size, time_steps, 5] where 3 for x, y, yaw, v, w
            ref_traj: np.array of shape [ref_traj_size, 3] where 3 for x, y, yaw
            traj_lookahead: int  
            goal_idx: int
            goals_interval: float
            desired_v: float

        Return:
            costs: np.array of shape [batch_size]
        """

        v = state[:, :, 3]

        beg = goal_idx - 1 if goal_idx != 0 else 0
        end = min(goal_idx + traj_lookahead, len(ref_traj))
        ref = ref_traj[beg:end, :3]

        costs = lin_vel_cost(v, desired_v)
        costs += ang_cost(state[:,:, 2:3], ref[:, 2])
        costs += self.triangle_cost_segments(state, ref, goals_interval)

        return costs

    def triangle_cost_segments(self, state, ref, goals_interval):
        x_dists = state[:, :, :1] - ref[:, 0]
        y_dists = state[:, :, 1:2] - ref[:, 1]
        dists = np.sqrt(x_dists**2 + y_dists**2)
        triangle_costs = np.empty(shape=(dists.shape[0], dists.shape[1], len(ref) - 1))

        if len(ref) == 1:
            return (dists.squeeze(2).sum(1))

        for q in range(len(ref) - 1):
            first_sides = dists[:, :, q]
            second_sides = dists[:, :, q + 1]
            opposite_sides = goals_interval
            triangle_costs[:, :, q] = self.triangle_cost_segment(opposite_sides, first_sides, second_sides)

        return triangle_costs.min(2).sum(1)


    def triangle_cost_segment(self, opposite_side, first_sides, second_sides):
        costs = np.empty(shape=first_sides.shape)

        first_obtuse_mask = self.is_angle_obtuse(first_sides, second_sides, opposite_side)
        second_obtuse_mask = self.is_angle_obtuse(second_sides, first_sides, opposite_side)
        h_mask = (~first_obtuse_mask) & (~second_obtuse_mask)

        costs[first_obtuse_mask] = second_sides[first_obtuse_mask]
        costs[second_obtuse_mask] = first_sides[second_obtuse_mask]
        costs[h_mask] = self.heron(opposite_side, first_sides[h_mask], second_sides[h_mask])

        return costs


    def is_angle_obtuse(self, opposite_side, b, c):
        return opposite_side**2 > (b**2 + c**2)


    def heron(self, opposite_side, b, c):
        p = (opposite_side + b + c) / 2.0
        h = 2.0 / opposite_side * np.sqrt(p * (p - opposite_side) * (p - b) * (p - c))
        return h


def lin_vel_cost(v, desired_v):
    DESIRED_V_WEIGHT = 1.0
    v_costs = DESIRED_V_WEIGHT * (v - desired_v).mean(1)**2
    return v_costs

def ang_cost(yaw, ref_yaw):
    YAW_WEIGHT = 0.45
    yaw_cost = YAW_WEIGHT * ((yaw - ref_yaw)**2).sum(2).sum(1)
    return yaw_cost


