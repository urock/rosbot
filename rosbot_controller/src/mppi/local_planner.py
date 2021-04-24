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
from utils.dtypes import Control
from utils.visualizations import visualize_reference

from optimizers.optimizer import Optimizer
from robot import Odom

import matplotlib.pyplot as plt


class LocalPlanner:

    def __init__(self, optimizer, metric):
        self.goal_tolerance = rospy.get_param('~local_planner/goal_tolerance', 0.2)
        self.traj_lookahead = rospy.get_param('~local_planner/traj_lookahead', 7)
        self.controller_freq = rospy.get_param('~local_planner/controller_freq', 90)

        self.rate = rospy.Rate(self.controller_freq)
        self.control_dt = 1.0 / self.controller_freq

        self.optimizer = optimizer
        self.odom = Odom()
        self.metric = metric

        self.controls = np.empty(shape=(0, 2))

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

                    self.controls = np.append(
                        self.controls, control.to_numpy()[np.newaxis], axis=0)
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
            self._show_metrics()
            return

    def _show_metrics(self):
        self.odom.tf_sub.unregister()

        offset = 1
        path_len = len(self.odom.path) - offset 
        lin_vels = self.odom.path[:-offset, 3]
        ang_vels = self.odom.path[:-offset, 4]

        value = self.metric(self.reference_traj, self.odom.path)
        rospy.loginfo("**************** Path Finished *****************\n")
        rospy.loginfo("Path Total Time: {:.6f}.".format(time() - self.path_arrive_time))
        rospy.loginfo("Path Error by {}: {:.6f}.".format(self.metric.__name__, value))
        rospy.loginfo("Mean velocities v = {:.6f}, w = {:.6f}.".
                      format(np.mean(lin_vels), np.mean(ang_vels)))

        path_rng = np.arange(path_len)
        plt.figure(1)
        plt.subplot(221)
        plt.plot(path_rng, lin_vels)
        plt.yscale('linear')
        plt.title('Linear velocitie')
        plt.xlabel('point')
        plt.ylabel('Linear vel')

        plt.subplot(222)
        plt.plot(path_rng, ang_vels)
        plt.yscale('linear')
        plt.title('Angular')
        plt.xlabel('point')
        plt.ylabel('Angular vel')


        control_len = len(self.controls)
        control_rng = np.arange(control_len)
        lin_controls = self.controls[:, 0]
        ang_controls = self.controls[:, 1]

        plt.subplot(223)
        plt.plot(control_rng, lin_controls)
        plt.yscale('linear')
        plt.title('Linear Control')
        plt.xlabel('point')
        plt.ylabel('Linear vel')

        plt.subplot(224)
        plt.plot(control_rng, ang_controls)
        plt.yscale('linear')
        plt.title('Angular Control')
        plt.xlabel('point')
        plt.ylabel('Angular vel')
        plt.show()

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
