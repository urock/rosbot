import numpy as np
import nnio
import time

import rospy
from typing import Callable

from utils.dtypes import State, Control, dist_L2, dist_L2_np
from utils.geometry import quaternion_to_euler
from utils.profiler import profile

from utils.visualizations import visualize_trajs, MarkerArray


class MPPIControler:
    def __init__(self, loss: Callable[[np.ndarray, np.ndarray, int, int], np.ndarray],
                 freq: float, v_std: float, w_std: float, limit_v: float,
                 traj_lookahead: int, iter_count: int, time_steps: int,
                 batch_size: int, model_path: str):

        self.limit_v = limit_v
        self.time_steps = time_steps
        self.traj_lookahead = traj_lookahead
        self.batch_size = batch_size
        self.iter_count = iter_count

        self.v_std = v_std
        self.w_std = w_std

        self.freq = freq
        self.model_path = model_path
        self.dt = 1.0 / self.freq

        self.model = nnio.ONNXModel(self.model_path)
        self.loss = loss

        self.control_matrix = np.zeros(shape=(self.batch_size, self.time_steps, 5))
        self.control_matrix[:, :, 4] = self.dt
        self.velocities = self.control_matrix[:, :, :2].view()  # v, w
        self.controls = self.control_matrix[:, :, 2:4].view()  # v, w
        self.curr_control = np.zeros(shape=(self.time_steps, 2))

        self.trajs_pub = rospy.Publisher('/mppi_trajs', MarkerArray, queue_size=10)

    def next_control(self, reference_traj, goal_idx, curr_state):
        best_control = None
        start = time.perf_counter()
        t = start
        for iter in range(self.iter_count):
            control_seqs = self.generate_next_control_seqs()
            self.update_init_state(curr_state, control_seqs)
            self.predict_velocities()
            trajectories = self.predict_trajectories(curr_state)

            losses = self.loss(trajectories, reference_traj, self.traj_lookahead, goal_idx)
            best_idx = np.argmin(losses, axis=0)
            best_loss = losses[best_idx]
            best_control = control_seqs[best_idx]

            visualize_trajs(0, self.trajs_pub, trajectories.view(), 0.3)
            t = time.perf_counter() - start
            self.curr_control = best_control

            if (best_loss <= 0.05):
                break

        v_best = self.curr_control[0 + round(t / self.dt), 0]
        w_best = self.curr_control[0 + round(t / self.dt), 1]

        rospy.loginfo_throttle(
            2, "Iter: {}. Exec Time {}.  [v, w] = [{:.2f} {:.2f}].  \n".format(iter + 1, t, v_best, w_best))
        return Control(v_best, w_best)

    def generate_next_control_seqs(self):
        """ 
        Return:
            Randomly generates control sequences with means in curr_control - 
            np.array of shape [batch_size, time_steps, 2] where 2 is for v, w respectively
        """
        v_noises = np.random.normal(0.0, self.v_std, size=(self.batch_size, self.time_steps, 1))
        w_noises = np.random.normal(0.0, self.w_std, size=(self.batch_size, self.time_steps, 1))
        noises = np.concatenate([v_noises, w_noises], axis=2)

        next_seqs = self.curr_control[np.newaxis] + noises
        next_seqs = np.clip(next_seqs, -self.limit_v, self.limit_v)  # Clip both v and w ?
        return next_seqs

    def update_init_state(self, curr_state, control_seqs):
        """ Updates current control_matrix velocities and controls

        Args:
            control_seqs: np.array of shape [batch_size, time_steps, 2] where 2 is for v, w 
        """
        self.velocities[:, 0, 0] = curr_state.v
        self.velocities[:, 0, 1] = curr_state.w
        self.controls[:, :] = control_seqs

    def predict_velocities(self):
        """ Fills in control_matrix with predicted velocities

        """
        time_steps = self.control_matrix.shape[1]
        for t_step in range(time_steps - 1):
            curr_batch = self.control_matrix[:, t_step].astype(np.float32)
            curr_predicted = self.model(curr_batch)
            self.velocities[:, t_step + 1] = curr_predicted

    def predict_trajectories(self, curr_state):
        """ Propagetes trajectories using control matrix velocities and current state

        Return:
            trajectory points - np.array of shape [batch_size, time_steps, 3] where 3 is for x, y, yaw respectively
        """
        v, w = self.velocities[:, :, 0], self.velocities[:, :, 1]
        current_yaw = curr_state.yaw
        yaw = np.cumsum(w * self.dt, axis=1)
        yaw += current_yaw - yaw[:, :1]
        v_x = v * np.cos(yaw)
        v_y = v * np.sin(yaw)
        x = np.cumsum(v_x * self.dt, axis=1)
        y = np.cumsum(v_y * self.dt, axis=1)
        x += curr_state.x - x[:, :1]
        y += curr_state.y - y[:, :1]

        traj_points = np.concatenate([
            x[:, :, np.newaxis],
            y[:, :, np.newaxis],
            yaw[:, :, np.newaxis],
        ], axis=2)
        return traj_points
