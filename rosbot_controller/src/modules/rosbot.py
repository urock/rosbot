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
        self.v_max = 5.0
        self.w_max = 1.5
        self.xy_margin_squared = 0.05



class Rosbot():


    def __init__(self):
        self.odom_state = RobotState()
        self.model_state = RobotState()
        self.params = Params()
        self.estimated_time = 0.0


    def set_odom_state(self, odom_state):
        self.odom_state = odom_state


    def dist_to_goal_L2(self, goal):             
        """
        param goal - Goal Class object
        """
        return  (goal.x - self.odom_state.x)**2 + (goal.y - self.odom_state.y)**2
            

    def goal_reached(self, goal):
        # rospy.logwarn("checking goal: x -> {:.2f}, y -> {:.2f}".format(goal.x, goal.y))
        return self.dist_to_goal_L2(goal) <= self.params.xy_margin_squared

    def calculate_contol(self, goal):
        #rospy.logwarn("calc control: state -> ({:.2f}, {:.2f}) goal -> ({:.2f}, {:.2f})".
                        # format(self.state.x, self.state.y, goal.x, goal.y))
        rho = self.dist_to_goal_L2(goal)

        
        azim_goal = math.atan2((goal.y - self.odom_state.y),(goal.x - self.odom_state.x))
        alpha = azim_goal - self.odom_state.yaw                                        

        if (abs(alpha) > math.pi): 
            alpha -= np.sign(alpha) * 2 * math.pi

        v = self.params.v_max * math.tanh(rho) * math.cos(alpha)
        if rho == 0:
            w = 0.0
        else:
            w = self.params.w_max * alpha + math.tanh(rho)*math.sin(alpha)*math.cos(alpha)/rho
        
        return v, w

    def control_AS (self, CurrentGoal):
        X1 = CurrentGoal.x - self.odom_state.x
        Y1 = CurrentGoal.y - self.odom_state.y
        X2 = X1*math.cos(self.odom_state.yaw) + Y1*math.sin(self.odom_state.yaw)
        Y2 = -X1*math.sin(self.odom_state.yaw)+Y1*math.cos(self.odom_state.yaw)
        
        if X2 == 0:
            Angle = 0
        else:
            Angle = math.atan2(Y2,X2)

        dist = np.sqrt(Y2*Y2 + X2*X2)

        return dist, Angle

    def PID(self, w_prev, ang_error_prev, dist, angle_error):
        
        Ki = 0.1
        Kp = 2.8          

        if abs(angle_error)>1.57:
            komp = -1.0
        else:
            komp = 0.5*(1 - angle_error/1.57)
                    
        v = (1.0+komp)* math.tanh(dist)

        w = w_prev+Kp*angle_error+(Ki*0.1-Kp)*ang_error_prev 

        Wmax = 7.33 

        if w>Wmax:
            w = Wmax
        elif w < -Wmax:
            w = -Wmax
        else:
            pass
        
        t1 = rospy.Time.now().to_sec()

        if angle_error>1.0: 
            self.estimated_time = 1.5+t1

        if t1<self.estimated_time and abs(angle_error)>0.05:
            Vmax = 0.1
        elif t1<self.estimated_time: 
            Vmax = 0.25
        else:
            Vmax = 1.0
        

        if v>Vmax:
            v = Vmax
        elif v < -Vmax:
            v = -Vmax
        else:
            pass

        return v, w

        

    def update_model_state(self, v, w, freq):
       
        if w == 0.0:
            rho = 0
        else:
            rho = v/w
     
        
        # step 1. Calc new robot position relative to its previous pose
        
        dt = 1.0 / freq

        x_r = rho * math.sin(w*dt)
        y_r = rho * (1 - math.cos(w*dt))

        # step 2. Transfrom this point to map fixed coordinate system taking into account current robot pose

        self.model_state.x += x_r * math.cos(self.model_state.yaw) - y_r * math.sin(self.model_state.yaw)
        self.model_state.y += x_r * math.sin(self.model_state.yaw) + y_r * math.cos(self.model_state.yaw)
        self.model_state.yaw +=  w * dt

        # self.model_state.x += x_r * math.cos(self.odom_state.yaw) - y_r * math.sin(self.odom_state.yaw)
        # self.model_state.y += x_r * math.sin(self.odom_state.yaw) + y_r * math.cos(self.odom_state.yaw)
        # self.model_state.yaw =  self.odom_state.yaw + w * dt

        return self.model_state


