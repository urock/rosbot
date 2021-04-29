#!/usr/bin/env python3

from copy import copy
import rospy
from time import time, perf_counter
import numpy as np
from utils.visualizations import visualize_trajs, visualize_traj, MarkerArray


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

        self._trajectories_pub = rospy.Publisher('/mppi_trajs', MarkerArray, queue_size=10)

    def start(self):
        self.optimizer.generator.state = self.odom.state
        self.goal_handler.state = self.odom.state

        try:
            while not rospy.is_shutdown():
                if self.path_handler.has_path:
                    self.goal_handler.reference_trajectory = self.path_handler.path
                    self.optimizer.reference_trajectory = self.path_handler.path
                    self.optimizer.reference_intervals = self.path_handler.path_intervals
                elif not self.goal_handler.path_finished:
                    start = perf_counter()
                    goal_idx = self.goal_handler.update_goal()
                    if not self.goal_handler.path_finished:
                        control = self.optimizer.calc_next_control(goal_idx)
                        self.metric_handler.add_control(copy(control))
                        self.controller.publish_control(control)

                        if self._visualize_trajs:
                            visualize_trajs(0, self._trajectories_pub,
                                            self.optimizer.curr_trajectories, 0.1)
                            visualize_traj(99999, self._trajectories_pub,
                                           self.optimizer.generator.propagete_curr_trajectory(), 0.05)

                        self.metric_handler.add_state(copy(self.odom.state))
                        t = perf_counter() - start

                        if self._wait_full_step:
                            rospy.sleep(self.optimizer.get_offset_time() - t)

                    else:
                        self.controller.publish_stop_control()
                        self.metric_handler.show_metrics(time() - self.path_handler.path_come_time,
                                                         self.optimizer.reference_trajectory)

        except KeyboardInterrupt:
            rospy.loginfo("Interrupted")
