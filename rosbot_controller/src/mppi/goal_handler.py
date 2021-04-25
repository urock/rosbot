import rospy
import numpy as np

class GoalHandler:
    def __init__(self):
        self.state: np.array 
        self.trajectory: np.array 
        self.goal_idx = 0 

        self.trajectory_lookahead = rospy.get_param("~goal_handler/traj_lookahead", 7)
        self.goal_tolerance = rospy.get_param("~goal_handler/goal_tolerance", 0.2)

    def set_state(self, state):
        self.state = np.array([state.x, state.y])

    def set_trajectory(self, trajectory):
        self.trajectory = trajectory

    def reset_goal_idx(self):
        self.goal_idx = 0

    def update_goal(self):
        dists = self._get_distances_to_trajectory_points()
        nearest_idx = np.argmin(dists) + self.goal_idx
        dist_to_nearest = dists[nearest_idx]

        if not self._is_goal_reached(dist_to_nearest):
            self.goal_idx = nearest_idx
        else:
            self.goal_idx = nearest_idx + 1

        return self.goal_idx

    def _get_distances_to_trajectory_points(self):
        end = self._get_last_considered_trajectory_idx()
        trajectory_considered = self.trajectory[self.goal_idx:end, :2]
        dists = np.sqrt(((self.state - trajectory_considered)**2).sum(1))
        return dists


    def _get_last_considered_trajectory_idx(self):
        traj_end = len(self.trajectory)
        end = self.goal_idx + self.trajectory_lookahead + 1
        return min(end, traj_end)

    def _is_goal_reached(self, dist):
        return dist < self.goal_tolerance

