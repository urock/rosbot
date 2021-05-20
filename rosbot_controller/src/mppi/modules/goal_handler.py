import rospy
import numpy as np


class GoalHandler:
    def __init__(self):
        self._get_params()

        self.state = None
        self.path_finished = True
        self.goal_idx = 0
        self.count_ahead: int

        self._reference_trajectory: np.array

    def _get_params(self):
        self._goal_tolerance = rospy.get_param("~goal_handler/goal_tolerance", 0.2)

    @property
    def reference_trajectory(self):
        return self._reference_trajectory

    @reference_trajectory.setter
    def reference_trajectory(self, trajectory):
        self._reference_trajectory = trajectory
        self.path_finished = False
        self.goal_idx = 0

    def update_goal(self):
        dists = self._get_distances_to_trajectory_points()
        nearest_idx = np.argmin(dists)
        dist_to_nearest = dists[nearest_idx]
        nearest_idx = nearest_idx + self.goal_idx

        if not self._is_goal_reached(dist_to_nearest):
            self.goal_idx = nearest_idx
        else:
            self.goal_idx = nearest_idx + 1

        if self.goal_idx == len(self._reference_trajectory):
            self.path_finished = True

        return self.goal_idx

    def _get_distances_to_trajectory_points(self):
        point = self.state.to_numpy()[: 2]
        end = self._get_last_considered_trajectory_idx()
        trajectory_considered = self._reference_trajectory[self.goal_idx:end, :2]
        dists = np.sqrt(((point - trajectory_considered)**2).sum(1))

        return dists

    def _get_last_considered_trajectory_idx(self):
        traj_end = len(self._reference_trajectory)
        end = self.goal_idx + self.count_ahead + 1
        return min(end, traj_end)

    def _is_goal_reached(self, dist):
        return dist < self._goal_tolerance
