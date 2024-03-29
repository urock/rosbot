#!/usr/bin/env python
# license removed for brevity
import rospy
import math
import tf2_ros
import tf
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from datetime import datetime
import matplotlib.pyplot as plt
import numpy as np
import os
import time as time_
from logger.logger_tools import plot_xy_data, plot_data, save_plot, write_to_file, show_graph


class Logger:
    """Class for visualization in the form of graphs (also saves to text file)
     the state of the robot 
     the state of the model 
     the specified trajectory (path)
     Control action (cmd_vel)
     """

    def __init__(self):
        rospy.init_node("logger", anonymous=True)

        # if true -> show plots on shutdown
        self.show_plots = rospy.get_param("~show_plots", False)
        # if true -> save data with the start time in the name
        self.track_time = rospy.get_param("~track_time", False)
        # path to the output data folder
        self.module_path = rospy.get_param("~output_file")
        # output folder name
        self.output_folder = rospy.get_param("~output_folder")
        
        self.parent_frame = rospy.get_param("~parent_frame", 'odom')
        self.robot_frame = rospy.get_param("~robot_frame", 'base_link')
        self.kinetic_model_frame = rospy.get_param("~kinetic_model_frame", 'model_link')
        self.nn_model_frame = rospy.get_param("~nn_model_frame", 'nn_model_link')

        self.timeout = int(rospy.get_param("~timeout", 0))

        self.module_path = self.module_path + self.output_folder
        if self.track_time:
            self.module_path += '/' + str(datetime.now().strftime('%Y-%m-%d-%H:%M:%S'))

        # declare tf buffer and listener for working with TF transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # node init time
        self.init_time = None
        #
        self.prev_time = None
        # flag for the first callback (used in Tf callback)
        self.first_tick = True
        self.fisrt_fill_state = True
        # container for trajectory (/path) fron path_publisher node
        self.trajectory = {'x': [], 'y': []}
        # container for /cmd_vel (input control) from path_follower node
        self.control = {'x': [], 'yaw': []}
        # container for robot state
        self.robot_state = {'x': [], 'y': [], 'yaw': [], 'v': [], 'w': []}
        # container for kinetic model state
        self.kinetic_model_state = {'x': [], 'y': [], 'yaw': [], 'v': [], 'w': []}
        # container for nn model state
        self.nn_model_state = {'x': [], 'y': [], 'yaw': [], 'v': [], 'w': []}
        # time spent running the simulator (for automated tests)
        self.delta_time = {'dt': []}
        self.time = {'t': []}
        self.time_spent = 0
        self.current_control = list()
        # declare subscribers
        self.trajectory_sub = rospy.Subscriber("/path", Path, self.path_callback)
        self.control_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        # self.timer_ = rospy.Timer(rospy.Duration(0.033), self.timer_callback)
        self.tf_sub = rospy.Subscriber('/tf', Twist, self.tf_callback)
        if self.timeout > 0:
            rospy.Timer(rospy.Duration(1), self.timeout_callback)

        # set the function that will be executed when shutdown
        rospy.on_shutdown(self.on_shutdown)

    def tf_callback(self, msg):
        for item in msg.transforms:
            if (
                item.header.frame_id == self.parent_frame and 
                item.child_frame_id == self.robot_frame   and 
                len(self.current_control) > 0
            ):
                current_time = time_.time()
                
                self.time['t'].append(current_time - self.init_time)
                # To calculate speed, but not for predict in the NN model
                self.delta_time['dt'].append(current_time - self.prev_time) 
                # print(self.delta_time['dt'])
                self.control['x'].append(self.current_control[0])
                self.control['yaw'].append(self.current_control[1])

                # self.fill_state(dst_frame='base_link', state=self.robot_state)
                self.get_states()
                self.prev_time = current_time

    def timeout_callback(self, time_event):
        """called every second, calculates the time spent on simulation,
        if it is more than the timeout, turns off the path follower nodes for the rosbot"""

        self.time_spent += 1
        if self.time_spent > self.timeout:
            os.popen("rosnode kill /model_runner")
            os.popen("rosnode kill /model_follower")

    def path_callback(self, msg):
        """stores path messages in a separate container"""

        for p in msg.poses:
            x, y = p.pose.position.x, p.pose.position.y
            self.trajectory['x'].append(x)
            self.trajectory['y'].append(y)

    def cmd_vel_callback(self, msg):
        """stores control messages in a separate container"""

        if self.first_tick:
            self.first_tick = False
            self.init_time = time_.time()
            self.prev_time = time_.time()        
        self.current_control = list([msg.linear.x, msg.angular.z])


    def get_states(self):
        """stores msg data about robot state
         and model state in separate containers"""
        # if it is the first callback set init time
        # s = time_.time()
        self.fill_state(dst_frame=self.robot_frame, state=self.robot_state, src_frame=self.parent_frame)
        self.fill_state(dst_frame=self.kinetic_model_frame, state=self.kinetic_model_state, src_frame=self.parent_frame)
        self.fill_state(dst_frame=self.nn_model_frame, state=self.nn_model_state, src_frame=self.parent_frame)
        # print("EXECUTION TIME  = {}".format(time_.time() - s))

    def fill_state(self, dst_frame, state, src_frame='odom'):
        """Receives the state of the model or robot via TF transformation"""
        # print("fill state", dst_frame)
        try:
            tf_transform = self.tf_buffer.lookup_transform(src_frame, dst_frame, rospy.Time(),
                                                           rospy.Duration(0.1)).transform

            trans_vec = tf_transform.translation
            rot_quat = tf_transform.rotation
            yaw = euler_from_quaternion([rot_quat.x, rot_quat.y, rot_quat.z, rot_quat.w])[2]
            state['x'].append(trans_vec.x) # append new x, y, yaw
            state['y'].append(trans_vec.y)
            state['yaw'].append(yaw)

            if dst_frame is self.kinetic_model_frame:
                state['v'].append(self.current_control[0])
                state['w'].append(self.current_control[1])
            else:
                if len(state['x'])==1:
                    v, w = 0, 0
                    state['v'].append(v)
                    state['w'].append(w)
                    self.fisrt_fill_state = False
                else:
                    dt = self.delta_time['dt'][-1]
                    x_new = trans_vec.x
                    y_new = trans_vec.y
                    yaw_new = yaw
                    x_prev = state['x'][-2]
                    y_prev = state['y'][-2]
                    yaw_prev = state['yaw'][-2]
                    # print(dt, x_new, y_new, yaw_new, x_prev, y_prev, yaw_prev)

                    d_yaw = yaw_new - yaw_prev
                    d_yaw = (d_yaw + math.pi) % (2 * math.pi) - math.pi
                    w = d_yaw / dt

                    vx = (x_new - x_prev) / dt
                    vy = (y_new - y_prev) / dt
                    v = math.sqrt(vx**2 + vy**2)

                    alpha = math.atan2(vy,vx)
                    v = v * math.cos(alpha - yaw_new)
                    # print("___________")
                    # print(v, w)
                    state['v'].append(v)
                    state['w'].append(w)

            

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return tf.LookupException

    def process_collected_data(self, name, data, plot_type=None, plot_names=None):
        """Builds and saves a graph from data,
          saves data to an output file """
        path = self.module_path + '/'
        # print(path)
        write_to_file(path=path, data=data, file_name=name)
        if plot_type is not None: # [['x', 'y']]
            plt.figure(name)
            fig, ax = plt.subplots(len(plot_type))
            i = 0
            for keys, plot_name in zip(plot_type, plot_names):
                if len(keys) == 2:
                    plot_xy_data(x=data[keys[0]], y=data[keys[1]], ax=ax[i], plot_name=plot_name)
                if len(keys) == 1:
                    plot_data(data[keys[0]], ax=ax[i], plot_name=plot_name)
                i += 1
            # path = self.module_path + '/'
            path = self.module_path + '/pictures'
            save_plot(path=path, name=name)
            

    def build_general_graph(self, data, folder):
        """Build a graph containing information about what the trajectory was,
         how the robot (robot state) passed it and how the model (model state) passed it """

        # TODO make a universal function

        folder_path = self.module_path + folder

        x1, y1 = np.array(data[0]['x']), np.array(data[0]['y'])
        x2, y2 = np.array(data[1]['x']), np.array(data[1]['y'])
        x3, y3 = np.array(data[2]['x']), np.array(data[2]['y'])
        x4, y4 = np.array(data[3]['x']), np.array(data[3]['y'])

        plt.rcParams.update({'font.size': 14})  # font size
        plt.rcParams['figure.figsize'] = (11.0, 8.0)
        plt.figure("trajectory and states")

        plt.plot(x1, y1, color='b', label='ground truth', linewidth=3)
        plt.plot(x4, y4, color='g', label='trajectory', linewidth=3)
        if len(x2) > 0 and len(y2) > 0:
            plt.plot(x2, y2, color='r', label='kinematic model', linewidth=3)
        if len(x3) > 0 and len(y3) > 0:
            plt.plot(x3, y3, color='yellow', label='NN model', linewidth=3)

        plt.xlabel('X')
        plt.ylabel('Y')
        plt.legend(loc='best')  # or loc=1
        plt.grid(True)
        save_plot(folder_path, name='general_graph', fmt='png')

    def plot_velocities_and_control(self):

        fig, ax = plt.subplots(2)
        ax[0].set_ylabel('m/s')
        ax[0].set_title("linear velocity and control")
        plot_xy_data(x=self.time['t'], y=self.robot_state['v'], ax=ax[0], plot_name="robot v")
        plot_xy_data(x=self.time['t'], y=self.control['x'], ax=ax[0], plot_name="u1")
        if len(self.kinetic_model_state['v']):
            plot_xy_data(x=self.time['t'], y=self.kinetic_model_state['v'], ax=ax[0], plot_name="kinematic model v")
        if len(self.nn_model_state['v']) > 0:
            plot_xy_data(x=self.time['t'], y=self.nn_model_state['v'], ax=ax[0], plot_name="NN model v")

        ax[1].set_ylabel('m/s')
        ax[1].set_xlabel('t, sec')
        ax[1].set_title("angular velocity and control")
        plot_xy_data(x=self.time['t'], y=self.robot_state['w'], ax=ax[1], plot_name="robot w")
        plot_xy_data(x=self.time['t'], y=self.control['yaw'], ax=ax[1], plot_name="u2")
        if len(self.kinetic_model_state['w']):
            plot_xy_data(x=self.time['t'], y=self.kinetic_model_state['w'], ax=ax[1], plot_name="kinematic model w")
        if len(self.nn_model_state['w']):
            plot_xy_data(x=self.time['t'], y=self.nn_model_state['w'], ax=ax[1], plot_name="NN model w")

        path = self.module_path + '/pictures'
        save_plot(path=path, name="velocities_and_control")

    def plot_yaw_over_time(self):

        fig3, ax3 = plt.subplots(1)
        ax3.set_xlabel('t, sec')        
        ax3.set_ylabel('Rads')
        ax3.set_title("Yaw angle over time")
        plot_xy_data(x=self.time['t'], y=self.robot_state['yaw'], ax=ax3, plot_name="yaw(t)")
        if len(self.kinetic_model_state['yaw']) > 0:
            plot_xy_data(x=self.time['t'], y=self.kinetic_model_state['yaw'], ax=ax3, plot_name="kinematic model yaw(t)")
        if len(self.nn_model_state['yaw']) > 0:
            plot_xy_data(x=self.time['t'], y=self.nn_model_state['yaw'], ax=ax3, plot_name="NN model yaw(t)")

        path = self.module_path + '/pictures'
        save_plot(path=path, name="yaw_over_time")       

    def plot_XY_over_time(self):
        fig2, ax2 = plt.subplots(2)
        ax2[0].set_ylabel('m')
        ax2[1].set_ylabel('m')
        ax2[1].set_xlabel('t, sec')    
        ax2[0].set_title("X coord over time")
        plot_xy_data(x=self.time['t'], y=self.robot_state['x'], ax=ax2[0], plot_name="robot x(t)")
        if len(self.kinetic_model_state['x']) > 0:
            plot_xy_data(x=self.time['t'], y=self.kinetic_model_state['x'], ax=ax2[0], plot_name="kinematic model x(t)")
        if len(self.nn_model_state['x']) > 0:
            plot_xy_data(x=self.time['t'], y=self.nn_model_state['x'], ax=ax2[0], plot_name="NN model x(t)")

        ax2[1].set_title("Y coord over time")
        plot_xy_data(x=self.time['t'], y=self.robot_state['y'], ax=ax2[1], plot_name="robot y(t)")
        if len(self.kinetic_model_state['y']) > 0:
            plot_xy_data(x=self.time['t'], y=self.kinetic_model_state['y'], ax=ax2[1], plot_name="kinematic model y(t)")

        if len(self.nn_model_state['y']) > 0:
            plot_xy_data(x=self.time['t'], y=self.nn_model_state['y'], ax=ax2[1], plot_name="NN model y(t)")

        path = self.module_path + '/pictures'
        save_plot(path=path, name="XY over time")   

        fig4, ax4 = plt.subplots(1)
        ax4.set_xlabel('X, m')        
        ax4.set_ylabel('Y, m')
        ax4.set_title("XY trajectory")
        plot_xy_data(x=self.robot_state['x'], y=self.robot_state['y'], ax=ax4, plot_name="x_y")
        if len(self.kinetic_model_state['y']) > 0:
            plot_xy_data(x=self.kinetic_model_state['x'], y=self.kinetic_model_state['y'], ax=ax4, plot_name="kinematic model x_y")
        if len(self.nn_model_state['y']) > 0:
            plot_xy_data(x=self.nn_model_state['x'], y=self.nn_model_state['y'], ax=ax4, plot_name="NN model x_y")

        save_plot(path=path, name="Y over X ") 



    def on_shutdown(self):
        """
        Process and save collected data
        """
        self.tf_sub.unregister()
        self.control_sub.unregister()
        data = [self.robot_state, self.kinetic_model_state, self.nn_model_state, self.trajectory]
        self.build_general_graph(data, '/pictures')
        # self.timer_.shutdown()

        self.plot_yaw_over_time()
        self.plot_XY_over_time()
        self.plot_velocities_and_control()
        # print(self.robot_state)
        self.process_collected_data(name='state', data=self.robot_state, plot_type=None)
        self.process_collected_data(name='kinetic_model_state', data=self.kinetic_model_state, plot_type=None)
        self.process_collected_data(name='nn_model_state', data=self.nn_model_state, plot_type=None)
        self.process_collected_data(name='time', data=self.time, plot_type=None)
        self.process_collected_data(name='control', data=self.control, plot_type=None)

        rospy.logwarn("Logger: output data folder - {}".format(self.module_path))

        if self.show_plots:
            show_graph()


def main():
    """ """
    logger = Logger()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.logerr("Shutting down")


if __name__ == "__main__":
    main()
