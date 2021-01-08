#!/usr/bin/env python
# license removed for brevity
import rospy
import os
import roslib
import tf
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist

from nav_msgs.msg import Path

from modules.rosbot import Rosbot, RobotState, RobotControl, Goal

# from visualization_msgs.msg import Marker


class TrajFollower():
    """
    TrajFollower tries to follow given path 
    Trajectory should be defined as array of (x, y, yaw ?) points

    It listens to robot_frame tf and computes control to follow the path
    """
    

    def __init__(self, node_name): 
        self.node_name = node_name
        rospy.init_node(self.node_name, anonymous=True)
        self.robot_frame = rospy.get_param('~robot_frame')
        self.cmd_topic = rospy.get_param('~cmd_topic')
        self.odom_frame = 'odom'

        self.robot = Rosbot()

        self.tf_listener = tf.TransformListener()
        self.tf_br = tf.TransformBroadcaster()

        self.cmd_freq = 10.0 # Hz       
        self.dt = 1.0 / self.cmd_freq  

        self.robot_state = RobotState()     
        self.current_goal = Goal()               

        self.goal_queue = []
        self.path = []

        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)
        self.path_sub = rospy.Subscriber("/path", Path, self.path_callback)
        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=5)
        
        self.rate = rospy.Rate(self.cmd_freq)
        self.path_index = 0
        self.got_path = False


    def get_robot_state_from_tf(self):
        try:
            (coord,orient) = self.tf_listener.lookupTransform(self.odom_frame, self.robot_frame, rospy.Time())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # rospy.logerr("Error in lookup transform from {} to {}".format(self.odom_frame, self.robot_frame))    
            return None

        x, y = coord[0], coord[1]
        yaw = tf.transformations.euler_from_quaternion(orient)[2]

        # vx = (x   - self.robot_state.x) * self.cmd_freq
        # vy = (y   - self.robot_state.y) * self.cmd_freq
        # w  = (yaw - self.robot_state.yaw) * self.cmd_freq

        return RobotState(x, y, yaw)

    def print_state(self, state):
        rospy.loginfo(state.to_str())

    def goal_callback(self, msg):
        x, y = msg.pose.position.x, msg.pose.position.y            
        self.goal_queue.append(Goal(x, y))
        rospy.logwarn("added goal to queue: x -> {:.2f}, y -> {:.2f}".format(x, y))
        
    def path_callback(self, msg):
        for p in msg.poses:
            x, y = p.pose.position.x, p.pose.position.y
            self.goal_queue.append(Goal(x, y))
            self.path.append(Goal(x, y))
        self.got_path = True

    def wait_for_path(self):
        while not rospy.is_shutdown():
            if self.got_path:
                break
            self.rate.sleep()


    def get_min_dist_to_path(self):

        lookback_index_dist = 10
        if self.path_index >= lookback_index_dist:
            path_slice = self.path[self.path_index - lookback_index_dist: self.path_index]
        else:
            path_slice = self.path[0: self.path_index]
        
        min_dist = 100
        for p in path_slice:
            dist = self.robot.dist_to_goal_L2(p)
            if dist < min_dist: 
                min_dist = dist
        return min_dist 
    
    def publish_control(self, control):
        """
        :param c: control vector of RobotControl type
        """
        twist_cmd = Twist()
        twist_cmd.linear.x = control.v
        twist_cmd.linear.y = 0
        twist_cmd.linear.z = 0
        twist_cmd.angular.x = 0
        twist_cmd.angular.y = 0
        twist_cmd.angular.z = control.w
        self.cmd_pub.publish(twist_cmd)

      

    def run(self):

        path_deviation = 0.0

        t0 = rospy.Time.now().to_sec()
        
        while not rospy.is_shutdown():
            self.robot_state = self.get_robot_state_from_tf()
            if self.robot_state is None:
                continue
            self.robot.set_state(self.robot_state)
        
            # if self.robot_frame == "model_link":
            #     rospy.logwarn(self.robot_frame + ": " + self.robot_state.to_str())

            # check if new goal should be selected and calc control
            if self.robot.goal_reached(self.current_goal):
                if self.goal_queue:
                    self.current_goal = self.goal_queue.pop(0)
                    self.path_index += 1
                    rospy.logerr(self.robot_frame + ": new current_goal = " + self.current_goal.to_str())
                else:
                    # end of trajectory
                    self.publish_control(RobotControl())    
                    self.rate.sleep()
                    break

            path_deviation = path_deviation + self.get_min_dist_to_path()
            # rospy.logwarn("path_deviation -> {:.2f}".format(path_deviation))

            control = self.robot.calculate_contol(self.current_goal)

            self.publish_control(control)

            self.rate.sleep()

        t1 = rospy.Time.now().to_sec()

        rospy.logwarn("Trajectory finished. Error -> {:.2f}, T -> {:.2f}".
                        format(path_deviation, t1-t0))
        # rospy.signal_shutdown("path ended")
        rospy.sleep(30.0)
        os.popen("rosnode kill /model_runner")
        # os.popen("rosnode kill /model_follower")
        os.popen("rosnode kill /plotter")
        return

def main():
    trajectory_follower = TrajFollower('trajectory_follower')
    trajectory_follower.wait_for_path()
    trajectory_follower.run()

if __name__ == '__main__':
    main()
