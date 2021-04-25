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


class MPPIOptimizer(Optimizer):
    def __init__(self, control_generator, model, cost, next_control_policy):
        self.control_generator = control_generator
        self.cost = cost
        self.next_control_policy = next_control_policy

        self.iter_count = int(rospy.get_param("~mppic/iter_count", 1))
        self.desired_v = rospy.get_param("~mppic/desired_v", 0.5)
        self.traj_lookahead = int(rospy.get_param("~mppic/traj_lookahead", 7))
        self.goals_interval = rospy.get_param("~mppic/goals_interval", 0.1)

        self.reference_traj: np.ndarray

    def set_reference_trajectory(self, ref_traj):
        self.reference_traj = ref_traj

    def update_state(self, state):
        self.control_generator.update_state(state)

    def get_next_control(self, goal_idx):
        start = perf_counter()
        for _ in range(self.iter_count):
            self._optimize(goal_idx)
        t = perf_counter() - start

        offset = round(t / self.dt)
        control = self._get_control(offset)

        rospy.loginfo_throttle(2, "Offset: {}. Goal: {}. Exec Time {:.4f}.  \nControl [v, w] = [{:.2f} {:.2f}]. \nOdom    [v, w] = [{:.2f} {:.2f}] \n".format(
            offset, goal_idx, t, control.v, control.w, self.control_generator.curr_state.v, self.control_generator.curr_state.w))

        self.control_generator.displace_controls(offset)
        return control

    def _optimize(self, state, goal_idx: int):
        state = self.control_generator.generate_next_controls()

        costs = self.cost(
            state,
            self.reference_traj,
            self.traj_lookahead,
            goal_idx,
            self.desired_v,
            self.goals_interval,
        )

        next_control_seq = self.next_control_policy(costs, self.control_generator.get_controls())
        self.control_generator.update_control_seq(next_control_seq)
        
    def _get_control(self, offset: int):
        offset = min(offset, self.time_steps - 1)
        v_best = self.control_generator.curr_control_seq[0 + offset, 0]
        w_best = self.control_generator.curr_control_seq[0 + offset, 1]
        return Control(v_best, w_best)

