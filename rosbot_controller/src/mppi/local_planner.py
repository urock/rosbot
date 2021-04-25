#!/usr/bin/env python3

import rospy

class LocalPlanner:
    def __init__(self, odom, optimizer, metric_handler, goal_handler, controller):
        self.optimizer = optimizer
        self.odom = odom
        self.goal_handler = goal_handler
        self.metric_handler = metric_handler
        self.controller = controller

    def start(self):
        """Starts main loop running mppi controller if got path."""
        self.optimizer.update_state(self.odom.curr_state)

        try:
            while not rospy.is_shutdown():
                self.optimizer.get_next_control()

        except KeyboardInterrupt:
            rospy.loginfo("Interrupted")
