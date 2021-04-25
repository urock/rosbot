#!/usr/bin/env python3

import rospy
from time import time
import numpy as np


class LocalPlanner:
    def __init__(self, optimizer, odom, controller, goal_handler, path_handler, metric_handler):
        self.optimizer = optimizer
        self.odom = odom
        self.controller = controller
        self.goal_handler = goal_handler
        self.path_handler = path_handler
        self.metric_handler = metric_handler

        self.reference_trajectory: np.ndarray

    def start(self):
        self.optimizer.set_state(self.odom.curr_state)
        self.goal_handler.set_state(self.odom.curr_state)

        try:
            while not rospy.is_shutdown():

                if self.path_handler.has_path:
                    self.reference_trajectory = self.path_handler.get_path()
                    self.optimizer.set_reference_trajectory(self.reference_trajectory)
                    self.goal_handler.set_reference_trajectory(self.reference_trajectory)

                if self.goal_handler.has_path:
                    goal_idx = self.goal_handler.update_goal()
                    if not self.goal_handler.path_finished:
                        control = self.optimizer.get_next_control(goal_idx)
                        self.controller.publish_control(control)
                    else:
                        self.metric_handler.show_metrics(time() - self.path_handler.path_come_time,
                                                         self.odom.path,
                                                         self.reference_trajectory, 2)
                else:
                    self.controller.publish_stop_control()

        except KeyboardInterrupt:
            rospy.loginfo("Interrupted")
