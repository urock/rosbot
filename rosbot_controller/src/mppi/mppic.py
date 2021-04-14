import numpy as np
import nnio
import time

import rospy
from typing import Callable, Type

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
        self.curr_control_seq = np.zeros(shape=(self.time_steps, 2))
        self.trajs_pub = rospy.Publisher('/mppi_trajs', MarkerArray, queue_size=10)

        self.reference_traj: np.ndarray
        self.curr_state: Type[State]

    def set_reference_traj(self, ref_traj):
        self.reference_traj = ref_traj

    def update_state(self, state: Type[State]):
        self.curr_state = state

    def next_control(self, goal_idx: int):
        start = time.perf_counter()
        for iter in range(self.iter_count):
            best_loss = self.optimize(goal_idx)

            if (best_loss <= 0.05):
                break
        t = time.perf_counter() - start

        control = self.get_control(round(t / self.dt))
        rospy.loginfo_throttle(
            2, "Iter: {}. Exec Time {}.  [v, w] = [{:.2f} {:.2f}].  \n".format(iter + 1, t, control.v, control.w))
        return control 

    def get_control(self, offset: int) -> Type[Control] :
        v_best = self.curr_control_seq[0 + offset  , 0]
        w_best = self.curr_control_seq[0 + offset, 1]

        return Control(v_best, w_best)

    def optimize(self, goal_idx: int):
        self.update_control_matrix()
        trajectories = self.predict_trajectories()
        losses = self.loss(trajectories, self.reference_traj, self.traj_lookahead, goal_idx)

        best_loss, best_control_seq = self.get_best_control_seq(losses)
        self.curr_control_seq = best_control_seq
        # visualize_trajs(0, self.trajs_pub, trajectories.view(), 0.3)
        return best_loss 


    def update_control_matrix(self):
        self.randomize_control_seqs()
        self.velocities[:, 0, 0] = self.curr_state.v
        self.velocities[:, 0, 1] = self.curr_state.w
        self.update_velocities()


    def get_best_control_seq(self, seqs_losses: np.ndarray):
        best_idx = np.argmin(seqs_losses, axis=0)
        best_loss = seqs_losses[best_idx]
        best_control = self.controls[best_idx]
        return best_loss, best_control


    def update_velocities(self):
        time_steps = self.control_matrix.shape[1]
        for t_step in range(time_steps - 1):
            curr_batch = self.control_matrix[:, t_step].astype(np.float32)
            curr_predicted = self.model(curr_batch)
            self.velocities[:, t_step + 1] = curr_predicted


    def randomize_control_seqs(self):
        """ 
        Return:
            Randomly generates control sequences with means in curr_control - 
            np.array of shape [batch_size, time_steps, 2] where 2 is for v, w respectively
        """
        v_noises = np.random.normal(0.0, self.v_std, size=(self.batch_size, self.time_steps, 1))
        w_noises = np.random.normal(0.0, self.w_std, size=(self.batch_size, self.time_steps, 1))
        noises = np.concatenate([v_noises, w_noises], axis=2)

        self.controls[:, :] = self.curr_control_seq[np.newaxis] + noises
        self.controls[:, :] = np.clip(self.controls[:, :], -self.limit_v, self.limit_v)  # Clip both v and w ?


    def predict_trajectories(self):
        """ Propagetes trajectories using control matrix velocities and current state

        Return:
            trajectory points - np.array of shape [batch_size, time_steps, 3] where 3 is for x, y, yaw respectively
        """
        v, w = self.velocities[:, :, 0], self.velocities[:, :, 1]
        current_yaw = self.curr_state.yaw
        yaw = np.cumsum(w * self.dt, axis=1)
        yaw += current_yaw - yaw[:, :1]
        v_x = v * np.cos(yaw)
        v_y = v * np.sin(yaw)
        x = np.cumsum(v_x * self.dt, axis=1)
        y = np.cumsum(v_y * self.dt, axis=1)
        x += self.curr_state.x - x[:, :1]
        y += self.curr_state.y - y[:, :1]

        traj_points = np.concatenate([
            x[:, :, np.newaxis],
            y[:, :, np.newaxis],
            yaw[:, :, np.newaxis],
        ], axis=2)
        return traj_points
