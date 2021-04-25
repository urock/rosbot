from utils.visualizations import visualize_trajs, MarkerArray
from utils.geometry import quaternion_to_euler
from utils.dtypes import State, Control
from optimizers.optimizer import Optimizer
import rospy
from time import perf_counter
from typing import Callable, Type
import numpy as np
import sys

sys.path.append("..")


class MPPIControlGenerator():
    def __init__(self, model):
        self.model = model

        self.freq = int(rospy.get_param("~mppic/mppi_freq", 30))
        self.dt = 1.0 / self.freq
        self.batch_size = int(rospy.get_param("~mppic/batch_size", 100))
        self.time_steps = int(rospy.get_param("~mppic/time_steps", 50))

        # Constraints
        self.v_std = rospy.get_param("~mppic/v_std", 0.1)
        self.w_std = rospy.get_param("~mppic/w_std", 0.1)
        self.limit_v = rospy.get_param("~mppic/limit_v", 0.5)
        self.limit_w = rospy.get_param("~mppic/limit_w", 0.7)

        # 5 for v, w, control_dim and dt
        self.batch_of_seqs = np.zeros(shape=(self.batch_size, self.time_steps, 5))
        self.batch_of_seqs[:, :, 4] = self.dt
        self.curr_control_seq = np.zeros(shape=(self.time_steps, 2))
        self.curr_state: Type[State]


    def set_state(self, state):
        self.curr_state = state


    def update_control_seq(self, control_seq):
        self.curr_control_seq = control_seq

    def generate_next_controls(self):
        self._update_batch_of_seqs()
        state = self._predict_trajs()
        return state

    def displace_controls(self, offset: int):
        if offset == 0:
            return

        control_cropped = self.curr_control_seq[offset:]
        end_part = np.array([self.curr_control_seq[-1]] * offset)
        self.curr_control_seq = np.concatenate([control_cropped, end_part], axis=0)

    def get_controls(self):
        return batch_of_seqs[:,:,2:4]

    def _update_batch_of_seqs(self, state, control_seq):
        noises = self._generate_noises()
        self.batch_of_seqs[:, 0, 0] = self.curr_state.v
        self.batch_of_seqs[:, 0, 1] = self.curr_state.w
        self.batch_of_seqs[:, :, 2:4] = self.curr_control_seq[None] + noises
        self.batch_of_seqs[:, :, 2:3] = np.clip(
            self.batch_of_seqs[:, :, 2:3], -self.limit_v, self.limit_v
        )
        self.batch_of_seqs[:, :, 3:4] = np.clip(
            self.batch_of_seqs[:, :, 3:4], -self.limit_w, self.limit_w
        )
        self._update_velocities()

    def _generate_noises(self):
        v_noises = np.random.normal(0.0, self.v_std, size=(self.batch_size, self.time_steps, 1))
        w_noises = np.random.normal(0.0, self.w_std, size=(self.batch_size, self.time_steps, 1))
        noises = np.concatenate([v_noises, w_noises], axis=2)

        return noises

    def _update_velocities(self):
        for t_step in range(self.time_steps - 1):
            curr_batch = self.batch_of_seqs[:, t_step].astype(np.float32)
            curr_predicted = self.model(curr_batch)
            self.batch_of_seqs[:, t_step + 1, :2] = curr_predicted

    # todo separate methods for this
    def _predict_trajs(self):
        """Propagetes trajectories using control matrix velocities and current state.

        Return:
            trajectory points - np.array of shape [batch_size, time_steps, 3] where 3 is for x, y, yaw respectively
        """
        v, w = self.batch_of_seqs[:, :, 0], self.batch_of_seqs[:, :, 1]
        current_yaw = self.curr_state.yaw
        yaw = np.cumsum(w * self.dt, axis=1)
        yaw += current_yaw - yaw[:, :1]
        v_x = v * np.cos(yaw)
        v_y = v * np.sin(yaw)
        x = np.cumsum(v_x * self.dt, axis=1)
        y = np.cumsum(v_y * self.dt, axis=1)
        x += self.curr_state.x - x[:, :1]
        y += self.curr_state.y - y[:, :1]

        traj_points = np.concatenate(
            [
                x[:, :, np.newaxis],
                y[:, :, np.newaxis],
                yaw[:, :, np.newaxis],
                self.batch_of_seqs[:, :, :1],
                self.batch_of_seqs[:, :, 1:2],
            ],
            axis=2,
        )

        return traj_points
