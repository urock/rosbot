#!/usr/bin/env python3

from copy import copy
import rospy
from utils.visualizations import StateVisualizer, TrajectoriesVisualizer, \
    ReferenceVisualizer, ObstaclesVisualizer, Colors

from geometry_msgs.msg import Vector3

from dynamic_reconfigure.server import Server
from rosbot_controller.cfg import MPPIConfig
import numpy as np


class LocalPlanner:
    def __init__(self, optimizer, odom, controller, goal_handler, path_handler, metric_handler):
        self._get_params()

        self.optimizer = optimizer
        self.odom = odom
        self.controller = controller
        self.goal_handler = goal_handler
        self.path_handler = path_handler
        self.metric_handler = metric_handler

        self.stop_robot = False
        self.stop_optimizer = False
        self.traj_vis_step = 10

        self._state_visualizer = StateVisualizer('/mppi_path')
        self._trajs_visualizer = TrajectoriesVisualizer('/mppi_trajs')
        self._ref_visualizer = ReferenceVisualizer('/ref_trajs')
        self._obstacle_visualizer = ObstaclesVisualizer('/mppi_obstacles')

        srv = Server(MPPIConfig, self.cfg_cb)

        rospy.sleep(1)
        self._obstacle_visualizer.visualize(self.optimizer.obstacles, Colors.red)

    def _get_params(self):
        self._visualize = rospy.get_param('~local_planner/visualize', False)
        self._wait_full_step = rospy.get_param('~local_planner/wait_full_step', False)

    def start(self):
        try:
            while not rospy.is_shutdown():
                if self.path_handler.has_path:
                    self._path_handle()
                elif not self.goal_handler.path_finished:
                    self._goal_handle()

        except KeyboardInterrupt:
            rospy.loginfo("Interrupted")

    def _path_handle(self):
        if self._visualize:
            self._ref_visualizer.reset()
            self._trajs_visualizer.reset()
            self._state_visualizer.reset()

        self.optimizer.generator.reset()
        self.optimizer.generator.state = self.odom.state
        self.goal_handler.state = self.odom.state

        self.goal_handler.reference_trajectory = self.path_handler.path
        self.optimizer.reference_trajectory = self.path_handler.path
        self.optimizer.reference_intervals = self.path_handler.path_intervals

    def _goal_handle(self):
        start = rospy.Time.now()

        self.goal_handler.count_ahead = self.optimizer.count_ahead
        goal_idx = self.goal_handler.update_goal()
        if not self.goal_handler.path_finished:
            control = None
            if not self.stop_optimizer:
                control = self.optimizer.calc_next_control(goal_idx)
                if not self.stop_robot:
                    self.controller.publish_control(control)

            self._visualizations_handle()
            t = (rospy.Time.now() - start).to_sec()

            if t < 3 and not self.stop_robot and control is not None:
                self._update_metrics(control)
                self.metric_handler.add_exec_time(t)
                rospy.loginfo("Local Planner. Mean Exec Time: {}\n".format(
                    self.metric_handler.get_mean_time()))
                rospy.loginfo("Local Planner. Std Exec Time: {}\n".format(
                    self.metric_handler.get_std_time()))

            if self._wait_full_step:
                sleep_time = self.optimizer.get_offset_time() - t
                rospy.sleep(sleep_time)
        else:
            self._path_finished_handle()

    def _path_finished_handle(self):
        self.controller.publish_stop_control()
        self.metric_handler.show_metrics((rospy.Time.now() - self.path_handler.path_come_time).to_sec(),
                                         self.optimizer.reference_trajectory, np.sum(self.optimizer.reference_intervals))

    def _visualizations_handle(self):
        if self._visualize:
            self._trajs_visualizer.add(self.optimizer.curr_trajectories,
                                       Colors.teal, scale=Vector3(0.025, 0.025, 0.025), step=self.traj_vis_step)
            self._trajs_visualizer.add([self.optimizer.generator.propagete_curr_trajectory()],
                                       Colors.red, scale=Vector3(0.05, 0.05, 0.05))

            self._trajs_visualizer.visualize()
            self._ref_visualizer.visualize(self.optimizer.reference_considered,
                                           Colors.purple, scale=Vector3(0.05, 0.05, 0.20))
            self._state_visualizer.visualize(
                self.odom.state, Colors.blue, scale=Vector3(0.025, 0.025, 0.025))

    def _update_metrics(self, control):
        self.metric_handler.add_state(copy(self.odom.state))
        self.metric_handler.add_control(copy(control))

    def cfg_cb(self, config, level):
        self.stop_optimizer = True

        if (config['stop_robot']):
            self.controller.publish_stop_control()

        rospy.sleep(1.5)

        self.traj_vis_step = config['traj_vis_step']
        self.optimizer.iter_count = config['iter_count']
        self.optimizer.traj_lookahead = config['traj_lookahead']
        self.optimizer.temperature = config['temperature']

        self.optimizer.generator.batch_size = config['batch_size']
        self.optimizer.generator.time_steps = config['time_steps']
        self.optimizer.generator.dt = config['model_dt']
        self.optimizer.generator.reset()

        if self._visualize:
            self._ref_visualizer.reset()
            self._trajs_visualizer.reset()

        self.optimizer.generator.v_std = config['v_std']
        self.optimizer.generator.w_std = config['w_std']

        self.optimizer.generator.limit_v = config['limit_v']
        self.optimizer.generator.limit_w = config['limit_w']
        self.optimizer.generator.limit_w = config['limit_w']

        self.optimizer.weights = {
            'goal': config['goal_weight'],
            'reference': config['reference_weight'],
            'obstacle': config['obstacle_weight']
        }

        self.optimizer.powers = {
            'goal': config['goal_power'],
            'reference': config['reference_power'],
            'obstacle': config['obstacle_power']
        }

        self._wait_full_step = config['wait_full_step']
        self._visualize = config['visualize']

        rospy.logwarn("Params changed")

        self.metric_handler.reset()
        self.stop_optimizer = False
        self.stop_robot = config['stop_robot']

        return config
