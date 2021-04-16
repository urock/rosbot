#!/usr/bin/env python3
import numpy as np
import nnio
from time import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray

from typing import Type

from utils.geometry import quaternion_to_euler
from utils.dtypes import Control, dist_L2_np
from utils.visualizations import visualize_reference

from robot import Odom
from mppic import MPPIControler

class LocalPlanner:
    def __init__(self, odom: Type[Odom], optimizer: Type[MPPIControler],
                 goal_tolerance: float, goals_interval: float):

        self.dt = 1.0 / optimizer.freq

        self.optimizer = optimizer
        self.odom = odom
        self.path_error = 0.0
        self.error_measurments_count = 0

        self.reference_traj = np.empty(shape=(0, 3))
        self.curr_goal_idx = - 1
        self.goal_tolerance = goal_tolerance
        self.goals_interval = goals_interval

        self.has_path = False
        self.path_arrive_time: float 
        self.path_sub = rospy.Subscriber("/path", Path, self.__path_cb)

        rospy.Timer(rospy.Duration(self.dt), self.__update_goal_cb)

        self.rate = rospy.Rate(self.optimizer.freq)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)

        self.ref_pub = rospy.Publisher('/ref_trajs', MarkerArray, queue_size=10)
        self.path_pub = rospy.Publisher("/mppi_path", Path, queue_size=5)

    def start(self):
        """Starts main loop running mppi controller if got path.
        """
        while not rospy.is_shutdown():
            if self.has_path:
                self.optimizer.update_state(self.odom.curr_state)
                control = self.optimizer.next_control(self.curr_goal_idx)
                self.rate.sleep()
                self.__publish_control(control)
            else:
                self.__publish_stop_control()
                self.rate.sleep()

    def __publish_control(self, control):
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

    def __publish_stop_control(self):
        self.__publish_control(Control())

    def __update_goal_cb(self, timer):
        if not self.has_path:
            return

        nearest_pt_idx, dist = self.__get_nearest_traj_point_and_dist()
        if not self.__is_goal_reached(dist):
            self.curr_goal_idx = nearest_pt_idx
        else:
            self.curr_goal_idx = nearest_pt_idx + 1

        if self.curr_goal_idx == len(self.reference_traj):
            self.has_path = False
            self.__print_metrics()
            return

        self.__update_error(dist)
        visualize_reference(2000, self.ref_pub, self.reference_traj,
                            self.curr_goal_idx, self.optimizer.traj_lookahead)

    def __print_metrics(self):
        rospy.loginfo("**************** Path Finished *****************\n")
        rospy.loginfo("Path Total Time: {:.6f}.".format(time() - self.path_arrive_time))
        rospy.loginfo("Path Mean Error: {:.6f}.".format(self.path_error / self.error_measurments_count))

    def __update_error(self, dist):
        self.path_error += dist
        self.error_measurments_count +=1


    def __get_nearest_traj_point_and_dist(self):
        min_idx = self.curr_goal_idx
        min_dist = 10e18

        traj_end = len(self.reference_traj)
        end = self.curr_goal_idx + self.optimizer.traj_lookahead + 1
        for q in range(self.curr_goal_idx, min(end, traj_end)):
            curr_dist = dist_L2_np(self.odom.curr_state, self.reference_traj[q])
            if min_dist >= curr_dist:
                min_dist = curr_dist
                min_idx = q

        return min_idx, min_dist

    def __is_goal_reached(self, dist):
        return dist < self.goal_tolerance

    def __path_cb(self, msg):
        for pose in msg.poses:
            x, y = pose.pose.position.x, pose.pose.position.y
            yaw = quaternion_to_euler(pose.pose.orientation)[0]
            self.reference_traj = np.append(self.reference_traj, [[x, y, yaw]], axis=0)

        self.optimizer.set_reference_traj(self.reference_traj)
        self.path_arrive_time = time()
        self.curr_goal_idx = 0
        self.has_path = True
