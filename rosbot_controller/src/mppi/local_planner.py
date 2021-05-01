#!/usr/bin/env python3

from copy import copy
import rospy
from time import time, perf_counter
from utils.visualizations import StateVisualizer, TrajectoriesVisualizer, ReferenceVisualizer, Colors

from geometry_msgs.msg import Vector3


class LocalPlanner:
    def __init__(self, optimizer, odom, controller, goal_handler, path_handler, metric_handler):
        self.optimizer = optimizer
        self.odom = odom
        self.controller = controller
        self.goal_handler = goal_handler
        self.path_handler = path_handler
        self.metric_handler = metric_handler

        self._visualize_trajs = rospy.get_param('~local_planner/visualize_trajs', False)
        self._wait_full_step = rospy.get_param('~local_planner/wait_full_step', False)

        self._state_visualizer = StateVisualizer('/mppi_path')
        self._trajs_visualizer = TrajectoriesVisualizer('/mppi_trajs')
        self._ref_visualizer = ReferenceVisualizer('/ref_trajs')

    def start(self):
        try:
            while not rospy.is_shutdown():
                if self.path_handler.has_path:
                    self._handle_new_path()
                elif not self.goal_handler.path_finished:
                    self._next_goal_handle()

        except KeyboardInterrupt:
            rospy.loginfo("Interrupted")

    def _handle_new_path(self):
        self._state_visualizer.reset()
        self._ref_visualizer.reset()
        self._trajs_visualizer.reset()
        self.optimizer.generator.reset()
        self.optimizer.generator.state = self.odom.state
        self.goal_handler.state = self.odom.state

        self.goal_handler.reference_trajectory = self.path_handler.path
        self.optimizer.reference_trajectory = self.path_handler.path
        self.optimizer.reference_intervals = self.path_handler.path_intervals

    def _next_goal_handle(self):
        start = perf_counter()
        goal_idx = self.goal_handler.update_goal()
        if not self.goal_handler.path_finished:
            self._next_control(goal_idx)
            if self._visualize_trajs:
                self._handle_visualizations()
            t = perf_counter() - start
            if self._wait_full_step:
                rospy.sleep(self.optimizer.get_offset_time() - t)
        else:
            self._handle_path_finished()

    def _next_control(self, goal_idx):
        control = self.optimizer.calc_next_control(goal_idx)
        self.controller.publish_control(control)
        self._update_metrics(control)

    def _handle_path_finished(self):
        self.controller.publish_stop_control()
        self.metric_handler.show_metrics(time() - self.path_handler.path_come_time,
                                         self.optimizer.reference_trajectory)

    def _handle_visualizations(self):

        self._trajs_visualizer.add(self.optimizer.curr_trajectories,
                                   Colors.teal, scale=Vector3(0.025, 0.025, 0.025), step=10)
        self._trajs_visualizer.add([self.optimizer.generator.propagete_curr_trajectory()],
                                   Colors.red, scale=Vector3(0.05, 0.05, 0.05))

        self._trajs_visualizer.visualize()
        self._ref_visualizer.visualize(self.optimizer.reference_considered,
                                       Colors.purple, scale=Vector3(0.05, 0.05, 0.20))

        self._state_visualizer.visualize(self.odom.state, Colors.blue, scale = Vector3(0.025, 0.025, 0.025))

    def _update_metrics(self, control):
        self.metric_handler.add_state(copy(self.odom.state))
        self.metric_handler.add_control(copy(control))
