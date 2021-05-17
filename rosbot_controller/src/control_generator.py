#!/usr/bin/env python
import rospy
import argparse
from geometry_msgs.msg import Twist
import time
import random
import numpy as np
import matplotlib.pyplot as plt
import os

def save_plot(path, name='', fmt='png'):
    """Saves graph to the output pkg"""

    pwd = os.getcwd()
    os.chdir(os.path.expanduser('~'))
    if not os.path.exists(path):
        os.makedirs(path)
    os.chdir(path)
    plt.savefig('{}.{}'.format(name, fmt), fmt='png')
    os.chdir(pwd)


class ControlGenerator():
    """
    ControlGenerator generates cmd_vel sequence
    - from file
    - periodic  
    """

    def __init__(self, node_name):
        self.node_name = node_name
        rospy.init_node(self.node_name, anonymous=True)
        self.mode = rospy.get_param('~control_mode')

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)

        self.dt = 0.03
        self.v = []
        self.w = []
        self.t = []
        rospy.on_shutdown(self.on_shutdown)

    def run(self):

        if self.mode == "from_file": 
            self.file_path = rospy.get_param('~file_path')

            self.read_control_from_file()

        elif self.mode == "periodic":
            self.Nt = int(((float)(rospy.get_param('~Tmax'))/self.dt))
            self.Tv = rospy.get_param('~period_lin')
            self.Tw = rospy.get_param('~period_ang')
            self.v_min = rospy.get_param('~v_min')
            self.v_max = rospy.get_param('~v_max')
            self.w_min = rospy.get_param('~w_min')
            self.w_max = rospy.get_param('~w_max')
            self.a_l = rospy.get_param('~a_lin')    # linear acceleration
            self.a_w = rospy.get_param('~a_ang')    # angular acceleration       

            self.generate_periodic_control()

        # self.build_graph(self.t, self.v, self.w)
        self.publish_control_sequence()


    def read_control_from_file(self):

        with open(self.file_path) as f:
            f.readline()
            lines = f.readlines()
            for i in range(len(lines[0:-1])):
                # print(lines[i].split(" "))
                cur_t, v, w = lines[i].rstrip().split(" ")
                cur_t, v, w = float(cur_t), float(v), float(w)
                self.t.append(cur_t)    
                self.v.append(v)
                self.w.append(w) 


    def publish_control_sequence(self):
        twist_cmd = Twist()
        for i, t in enumerate(self.t):
            twist_cmd.linear.x = self.v[i]
            twist_cmd.angular.z = self.w[i]

            self.cmd_pub.publish(twist_cmd)
            rospy.sleep(self.dt)

        twist_cmd.linear.x = 0.0
        twist_cmd.angular.z = 0.0

        self.cmd_pub.publish(twist_cmd)
        


    def generate_periodic_control(self):
        t = self.time_sequence()
        v = self.periodic_sequence_v()
        w = self.periodic_sequence_w()
        for i in range(self.Nt):
            self.t.append(next(t))    
            self.v.append(next(v))
            self.w.append(next(w))    
            
            
    def time_sequence(self):
        t = 0
        while True:
            yield t
            t += self.dt   


    def periodic_sequence_v(self):
        v = 0
        yield v
        for _ in range(int(self.Tv/(2*self.dt))):
            if self.a_l > 0 and v < self.v_max:
                v += self.a_l * self.dt         

            if self.a_l < 0 and abs(v) < self.v_max:
                v += self.a_l * self.dt

            yield v             

        while True:
            for _ in range(int(self.Tv/(2*self.dt))):
                if self.a_l > 0 and v > self.v_min:
                    v -= self.a_l * self.dt

                if self.a_l < 0 and v < self.v_min:
                    v -= self.a_l * self.dt

                yield v
            for _ in range(int(self.Tv/(2*self.dt))):
                if self.a_l > 0 and v < self.v_max:
                    v += self.a_l * self.dt         

                if self.a_l < 0 and abs(v) < self.v_max:
                    v += self.a_l * self.dt
                    
                yield v  
        

    def periodic_sequence_w(self):
        w = 0
        yield w
        for _ in range(int(self.Tw/(2*self.dt))):
            if self.a_w > 0 and w < self.w_max:
                w += self.a_w * self.dt         

            if self.a_w < 0 and abs(w) < self.w_max:
                w += self.a_w * self.dt

            yield w           

        while True:
            for _ in range(int(self.Tw/(2*self.dt))):
                if self.a_w > 0 and w > self.w_min:
                    w -= self.a_w * self.dt

                if self.a_w < 0 and w < self.w_min:
                    w -= self.a_w * self.dt

                yield w
            for _ in range(int(self.Tw/(2*self.dt))):
                if self.a_w > 0 and w < self.w_max:
                    w += self.a_w * self.dt         

                if self.a_w < 0 and abs(w) < self.w_max:
                    w += self.a_w * self.dt
                    
                yield w   

    def build_graph(self, t, v, w):

        t, v, w = np.array(t), np.array(v), np.array(w) 

        plt.rcParams.update({'font.size': 14})  # font size
        plt.rcParams['figure.figsize'] = (11.0, 8.0)
        plt.figure("control over time")

        plt.plot(t, v, color='b', label='u1', linewidth=3)
        plt.plot(t, w, color='r', label='u2', linewidth=3)

        plt.xlabel('t')
        plt.ylabel('u')
        plt.legend(loc='best')  # or loc=1
        plt.grid(True)
        plt.show()
        # save_plot(os.getcwd(), name='graph', fmt='png')

    def on_shutdown(self):
        os.popen("rosnode kill /logger")



def main():
    control_gen = ControlGenerator('control_generator')
    control_gen.run()

if __name__ == '__main__':
    main()
