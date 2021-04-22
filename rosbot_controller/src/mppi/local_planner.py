#!/usr/bin/env python3

import numpy as np
from time import time
from typing import Type
from copy import copy

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray

from utils.geometry import quaternion_to_euler
from utils.dtypes import Control, dist_L2_np
from utils.visualizations import visualize_reference


class LocalPlanner:
    def __init__(self, odom, optimizer, metric):
        self.goal_tolerance = rospy.get_param('~local_planner/goal_tolerance', 0.2)
        self.traj_lookahead = rospy.get_param('~local_planner/traj_lookahead', 7)
        self.controller_freq = rospy.get_param('~local_planner/controller_freq', 90)

        self.rate = rospy.Rate(self.controller_freq)
        self.control_dt = 1.0 / self.controller_freq

        self.optimizer = optimizer
        self.odom = odom
        self.metric = metric

        self.path_points = []
        self.lin_vels = []
        self.ang_vels = []

        self.reference_traj = np.empty(shape=(0, 3))
        self.curr_goal_idx = - 1

        self.has_path = False
        self.path_arrive_time: float
        self.path_sub = rospy.Subscriber("/path", Path, self._path_cb)

        rospy.Timer(rospy.Duration(self.control_dt), self._update_goal_cb)

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.ref_pub = rospy.Publisher('/ref_trajs', MarkerArray, queue_size=10)
        self.path_pub = rospy.Publisher('/mppi_path', Path, queue_size=5)

    def start(self):
        """Starts main loop running mppi controller if got path.
        """
        self.optimizer.update_state(self.odom.curr_state)

        try:
            while not rospy.is_shutdown():
                if self.has_path:
                    control = self.optimizer.next_control(self.curr_goal_idx)
                    self._publish_control(control)
                else:
                    self._publish_stop_control()
                self.rate.sleep()

        except KeyboardInterrupt:
            rospy.loginfo("Interrupted")

    def _publish_control(self, control):
        """ Publishes controls for the rosbot

        Args:
            control: control vector of Control type
        """
        cmd = Twist()
        cmd.linear.x = control.v
        cmd.linear.y = 0
        cmd.linear.z = 0
        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = control.w
        self.cmd_pub.publish(cmd)

    def _publish_stop_control(self):
        self._publish_control(Control())

    def _update_goal_cb(self, timer):
        if not self.has_path:
            return

        nearest_pt_idx, dist = self._get_nearest_traj_point_and_dist()
        if not self._is_goal_reached(dist):
            self.curr_goal_idx = nearest_pt_idx
        else:
            self.curr_goal_idx = nearest_pt_idx + 1

        if self.curr_goal_idx == len(self.reference_traj):
            self.has_path = False
            self._print_metrics()
            return

        self.path_points.append(copy(self.odom.curr_state))
        self.lin_vels.append(self.odom.curr_state.v)
        self.ang_vels.append(self.odom.curr_state.w)


    def _print_metrics(self):
        value = self.metric(self.reference_traj, self.path_points)
        rospy.loginfo("**************** Path Finished *****************\n")
        rospy.loginfo("Path Total Time: {:.6f}.".format(time() - self.path_arrive_time))
        rospy.loginfo("Path Error by {}: {:.6f}.".format(self.metric.__name__, value))
        rospy.loginfo("Mean velocities v = {:.6f}, w = {:.6f}.".format(np.sum(self.lin_vels) / len(self.lin_vels), 
            np.sum(self.ang_vels)/ len(self.ang_vels) ) )



    def _get_nearest_traj_point_and_dist(self):
        min_idx = self.curr_goal_idx
        min_dist = 10e18

        traj_end = len(self.reference_traj)
        end = self.curr_goal_idx + self.traj_lookahead + 1
        end = min(end, traj_end)
        for q in range(self.curr_goal_idx, end):
            curr_dist = dist_L2_np(self.odom.curr_state, self.reference_traj[q])
            if min_dist >= curr_dist:
                min_dist = curr_dist
                min_idx = q

        return min_idx, min_dist

    def _is_goal_reached(self, dist):
        return dist < self.goal_tolerance

    def _path_cb(self, msg):
        for pose in msg.poses:
            x, y = pose.pose.position.x, pose.pose.position.y
            yaw = quaternion_to_euler(pose.pose.orientation)[0]
            self.reference_traj = np.append(self.reference_traj, [[x, y, yaw]], axis=0)

        self.optimizer.set_reference_traj(self.reference_traj)
        self.path_arrive_time = time()
        self.curr_goal_idx = 0
        self.has_path = True
