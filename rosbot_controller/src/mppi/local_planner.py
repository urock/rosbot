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

        self.path_points = np.empty(shape=(0, 5))

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

        nearest_pt_idx, dist = self._get_nearest_ref_pt_and_dist()
        if not self._is_goal_reached(dist):
            self.curr_goal_idx = nearest_pt_idx
        else:
            self.curr_goal_idx = nearest_pt_idx + 1

        if self.curr_goal_idx == len(self.reference_traj):
            self.has_path = False
            self._print_metrics()
            return

        self.path_points = np.append(
            self.path_points, self.odom.curr_state.to_numpy()[np.newaxis], axis=0)

    def _print_metrics(self):
        value = self.metric(self.reference_traj, self.path_points)
        rospy.loginfo("**************** Path Finished *****************\n")
        rospy.loginfo("Path Total Time: {:.6f}.".format(time() - self.path_arrive_time))
        rospy.loginfo("Path Error by {}: {:.6f}.".format(self.metric.__name__, value))
        rospy.loginfo("Mean velocities v = {:.6f}, w = {:.6f}.".
                      format(np.mean(self.path_points[:, 3]), np.mean(self.path_points[:, 4])))

    def _get_nearest_ref_pt_and_dist(self):
        pt = np.array([self.odom.curr_state.x, self.odom.curr_state.y])
        beg = self.curr_goal_idx
        end = self._get_last_ref_considered_idx()
        ref_pts = self.reference_traj[beg:end, :2]

        dists = np.sqrt(((pt - ref_pts)**2).sum(1))
        idx = np.argmin(dists)

        return idx + self.curr_goal_idx, dists[idx]

    def _get_last_ref_considered_idx(self):
        traj_end = len(self.reference_traj)
        end = self.curr_goal_idx + self.traj_lookahead + 1
        return min(end, traj_end)

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
