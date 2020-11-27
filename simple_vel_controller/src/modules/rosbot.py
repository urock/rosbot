#!/usr/bin/env python
# license removed for brevity

import math
import numpy as np
import rospy

class RobotState():

    def __init__(self, x = 0.0, y = 0.0, yaw = 0.0, vx = 0.0, vy = 0.0, w = 0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.vx = vx
        self.vy = vy
        self.w = w


class Goal():

    def __init__(self, x = 0.0, y = 0.0):
        self.x = x
        self.y = y

class Params():

    def __init__(self):
        self.v_max = 0.4
        self.w_max = 0.4
        self.xy_margin_squared = 0.2



class Rosbot():


    def __init__(self):
        self.state = RobotState()
        self.params = Params()


    def set_odom_state(self, odom_state):
        self.state = odom_state


    def dist_to_goal_L2(self, goal):             
        """
        param goal - Goal Class object
        """
        return  (goal.x - self.state.x)**2 + (goal.y - self.state.y)**2 
            

    def goal_reached(self, goal):
        # rospy.logwarn("checking goal: x -> {:.2f}, y -> {:.2f}".format(goal.x, goal.y))
        return self.dist_to_goal_L2(goal) <= self.params.xy_margin_squared

    def calculate_contol(self, goal):
        #rospy.logwarn("calc control: state -> ({:.2f}, {:.2f}) goal -> ({:.2f}, {:.2f})".
                        # format(self.state.x, self.state.y, goal.x, goal.y))
        rho = self.dist_to_goal_L2(goal)

        
        azim_goal = math.atan2((goal.y - self.state.y),(goal.x - self.state.x))
        alpha = azim_goal - self.state.yaw                                        

        if (abs(alpha) > math.pi): 
            alpha -= np.sign(alpha) * 2 * math.pi

        v = self.params.v_max * math.tanh(rho) * math.cos(alpha)
        w = self.params.w_max * alpha + math.tanh(rho)*math.sin(alpha)*math.cos(alpha)/rho

        return v, w


