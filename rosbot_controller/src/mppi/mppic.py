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
    def __init__(self,
                 loss: Callable[[np.ndarray, np.ndarray, int, int], np.ndarray],
                 calc_next_control_seq_policie: Callable[[np.array, np.ndarray], np.array],
                 freq: float, v_std: float, w_std: float, limit_v: float, temperature: float,
                 traj_lookahead: int, iter_count: int, time_steps: int, batch_size: int,
                 model_path: str):

        self.calc_losses = loss
        self.calc_next_control_seq = calc_next_control_seq_policie

        self.limit_v = limit_v
        self.time_steps = time_steps
        self.traj_lookahead = traj_lookahead
        self.batch_size = batch_size
        self.iter_count = iter_count
        self.temperature = temperature

        self.v_std = v_std
        self.w_std = w_std

        self.freq = freq
        self.model_path = model_path
        self.dt = 1.0 / self.freq

        self.model = nnio.ONNXModel(self.model_path)

        # 5 for v, w, control_dim and dt
        self.batch_of_seqs = np.zeros(shape=(self.batch_size, self.time_steps, 5))
        self.batch_of_seqs[:, :, 4] = self.dt

        self.curr_control_seq = np.ones(shape=(self.time_steps, 2)) * 0.2
        self.trajs_pub = rospy.Publisher('/mppi_trajs', MarkerArray, queue_size=10)

        self.reference_traj: np.ndarray
        self.curr_state: Type[State]

    def set_reference_traj(self, ref_traj):
        self.reference_traj = ref_traj

    def update_state(self, state: Type[State]):
        self.curr_state = state

    def next_control(self, goal_idx: int):
        start = time.perf_counter()
        for _ in range(self.iter_count):
            self.__optimize(goal_idx)
        t = time.perf_counter() - start

        offset = round(round(t / self.dt))
        control = self.__get_control(offset)
        self.__displace_controls(offset)

        rospy.loginfo_throttle(2, "Offset: {}. Exec Time {}.  [v, w] = [{:.2f} {:.2f}].  \n".format(
            offset, t, control.v, control.w))
        return control

    def __optimize(self, goal_idx: int):
        # Update batch
        start = time.time()
        self.update_batch_of_seqs()
        end = time.time() - start
        rospy.loginfo_throttle(2, "Update batch_seqs {:.5f} ".format(end))

        # Predict trajectories
        start = time.time()
        trajectories = self.__predict_trajectories()
        end = time.time() - start
        rospy.loginfo_throttle(2, "Predict trajectories {:.5f}".format(end))

        # Calc losses
        start = time.time()
        losses = self.calc_losses(trajectories, self.reference_traj.view(),
                                  self.traj_lookahead, goal_idx)
        end = time.time() - start
        rospy.loginfo_throttle(2, "loss {:.5f}".format(end))

        # Calc next control
        start = time.time()
        next_control_seq = self.calc_next_control_seq(losses, self.batch_of_seqs[:, :, 2:4])
        end = time.time() - start
        rospy.loginfo_throttle(2, "calc_next_control_seq {:.5f}".format(end))

        self.curr_control_seq = next_control_seq

        visualize_trajs(0, self.trajs_pub, trajectories, 0.8)

    def update_batch_of_seqs(self):
        noises = self.__generate_noises()
        self.batch_of_seqs[:, 0, 0] = self.curr_state.v
        self.batch_of_seqs[:, 0, 1] = self.curr_state.w
        self.batch_of_seqs[:, :, 2:4] = self.curr_control_seq[None] + noises
        self.batch_of_seqs[:, :, 2:4] = np.clip(self.batch_of_seqs[:, :, 2:4], -self.limit_v,
                                                self.limit_v)  # Clip both v and w ?

        self.__update_velocities()

    def __generate_noises(self):
        v_noises = np.random.normal(0.0, self.v_std, size=(self.batch_size, self.time_steps, 1))
        w_noises = np.random.normal(0.0, self.w_std, size=(self.batch_size, self.time_steps, 1))
        noises = np.concatenate([v_noises, w_noises], axis=2)

        return noises

    def __update_velocities(self):

        for t_step in range(self.time_steps - 1):
            curr_batch = self.batch_of_seqs[:, t_step].astype(np.float32)
            curr_predicted = self.model(curr_batch)
            self.batch_of_seqs[:, t_step + 1, :2] = curr_predicted

    def __get_control(self, offset: int) -> Type[Control]:
        v_best = self.curr_control_seq[0 + offset, 0]
        w_best = self.curr_control_seq[0 + offset, 1]

        return Control(v_best, w_best)

    def __displace_controls(self, offset: int):
        if offset == 0:
            return

        control_cropped = self.curr_control_seq[offset:]
        end_part = np.array([self.curr_control_seq[-1]] * offset)
        self.curr_control_seq = np.concatenate([control_cropped, end_part], axis=0)

    def __predict_trajectories(self):
        """ Propagetes trajectories using control matrix velocities and current state

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

        traj_points = np.concatenate([
            x[:, :, np.newaxis],
            y[:, :, np.newaxis],
            yaw[:, :, np.newaxis],
        ], axis=2)
        return traj_points
