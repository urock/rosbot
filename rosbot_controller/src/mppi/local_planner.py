#!/usr/bin/env python3

import rospy
from time import time
import numpy as np
from utils.visualizations import visualize_trajs, MarkerArray


class LocalPlanner:
    def __init__(self, optimizer, odom, controller, goal_handler, path_handler, metric_handler):
        self.optimizer = optimizer
        self.odom = odom
        self.controller = controller
        self.goal_handler = goal_handler
        self.path_handler = path_handler
        self.metric_handler = metric_handler

        self._reference_trajectory: np.ndarray
        self._control = None

        self._is_visualize_trajs = True

        self._trajectories_pub = rospy.Publisher('/mppi_trajs', MarkerArray, queue_size=10)

    def start(self):
        self.optimizer.set_state(self.odom.curr_state)
        self.goal_handler.set_state(self.odom.curr_state)

        try:
            while not rospy.is_shutdown():

                if self.path_handler.has_path:
                    self._reference_trajectory = self.path_handler.get_path()
                    self.optimizer.set_reference_trajectory(self._reference_trajectory)
                    self.goal_handler.set_reference_trajectory(self._reference_trajectory)

                elif not self.goal_handler.path_finished:
                    goal_idx = self.goal_handler.update_goal()
                    if not self.goal_handler.path_finished:
                        control = self.optimizer.calc_next_control(goal_idx)
                        self.controller.publish_control(control)

                        if self._is_visualize_trajs:
                            visualize_trajs(0, self._trajectories_pub,
                                            self.optimizer.get_curr_trajectories())

                    else:
                        self.metric_handler.show_metrics(time() - self.path_handler.path_come_time,
                                                         self.odom.path,
                                                         self._reference_trajectory, 2)
                else:
                    self.controller.publish_stop_control()

        except KeyboardInterrupt:
            rospy.loginfo("Interrupted")
