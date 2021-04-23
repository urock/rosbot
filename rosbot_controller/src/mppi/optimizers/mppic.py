from utils.visualizations import visualize_trajs, MarkerArray
from utils.geometry import quaternion_to_euler
from utils.dtypes import State, Control, dist_L2, dist_L2_np
from optimizers.optimizer import Optimizer
import rospy
from time import perf_counter
from typing import Callable, Type
import numpy as np
import sys
sys.path.append("..")


class MPPIController(Optimizer):
    def __init__(self, model, cost, next_control_policy):
        self.model = model
        self.cost = cost
        self.next_control_policy = next_control_policy

        self.freq = int(rospy.get_param('~mppic/mppi_freq', 30))
        self.dt = 1.0 / self.freq
        self.batch_size = int(rospy.get_param('~mppic/batch_size', 100))
        self.time_steps = int(rospy.get_param('~mppic/time_steps', 50))
        self.iter_count = int(rospy.get_param('~mppic/iter_count', 1))

        self.v_std = rospy.get_param('~mppic/v_std', 0.1)
        self.w_std = rospy.get_param('~mppic/w_std', 0.1)
        self.limit_v = rospy.get_param('~mppic/limit_v', 0.5)
        self.limit_w = rospy.get_param('~mppic/limit_w', 0.7)
        self.desired_v = rospy.get_param('~mppic/desired_v', 0.5)
        self.traj_lookahead = int(rospy.get_param('~mppic/traj_lookahead', 7))

        self.goals_interval = rospy.get_param('~mppic/goals_interval', 0.1)

        # 5 for v, w, control_dim and dt
        self.batch_of_seqs = np.zeros(shape=(self.batch_size, self.time_steps, 5))
        self.batch_of_seqs[:, :, 4] = self.dt
        self.curr_control_seq = np.zeros(shape=(self.time_steps, 2))

        self.reference_traj: np.ndarray
        self.curr_state: Type[State]

        self.trajs_pub = rospy.Publisher('/mppi_trajs', MarkerArray, queue_size=10)

    def set_reference_traj(self, ref_traj):
        self.reference_traj = ref_traj

    def update_state(self, state: Type[State]):
        self.curr_state = state

    def next_control(self, goal_idx):
        start = perf_counter()
        for _ in range(self.iter_count):
            self._optimize(goal_idx)

        t = perf_counter() - start

        offset = round(t / self.dt)
        offset = min(offset, self.time_steps - 1)

        control = self._get_control(offset)

        rospy.loginfo_throttle(2, "Offset: {}. Goal: {}. Exec Time {:.4f}.  \nControl [v, w] = [{:.2f} {:.2f}]. \nOdom    [v, w] = [{:.2f} {:.2f}] \n".format(
            offset, goal_idx, t, control.v, control.w, self.curr_state.v, self.curr_state.w))

        self._displace_controls(offset)
        return control

    def _optimize(self, goal_idx: int):
        self._update_batch_of_seqs()
        state = self._predict_trajs()
        costs = self.cost(state, self.reference_traj, self.traj_lookahead,
                          goal_idx, self.desired_v, self.goals_interval)

        next_control_seq = self.next_control_policy(costs, self.batch_of_seqs[:, :, 2:4])
        self.curr_control_seq = next_control_seq
        visualize_trajs(0, self.trajs_pub, state, 0.8)

    def _update_batch_of_seqs(self):
        noises = self._generate_noises()
        self.batch_of_seqs[:, 0, 0] = self.curr_state.v
        self.batch_of_seqs[:, 0, 1] = self.curr_state.w
        self.batch_of_seqs[:, :, 2:4] = self.curr_control_seq[None] + noises
        self.batch_of_seqs[:, :, 2:3] = np.clip(self.batch_of_seqs[:, :, 2:3], -self.limit_v,
                                                self.limit_v)
        self.batch_of_seqs[:, :, 3:4] = np.clip(self.batch_of_seqs[:, :, 3:4], -self.limit_w,
                                                self.limit_w)
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

    def _get_control(self, offset: int):
        v_best = self.curr_control_seq[0 + offset, 0]
        w_best = self.curr_control_seq[0 + offset, 1]

        return Control(v_best, w_best)

    def _displace_controls(self, offset: int):
        if offset == 0:
            return

        control_cropped = self.curr_control_seq[offset:]
        end_part = np.array([self.curr_control_seq[-1]] * offset)
        self.curr_control_seq = np.concatenate([control_cropped, end_part], axis=0)

    def _predict_trajs(self):
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
            self.batch_of_seqs[:, :, :1],
            self.batch_of_seqs[:, :, 1:2]
        ], axis=2)

        return traj_points
