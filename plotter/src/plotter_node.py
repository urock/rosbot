#!/usr/bin/env python
# license removed for brevity
import rospy
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
from plotter.plotter_tools import plot_xy_data, plot_data, save_plot, write_to_file, show_graph


class Plotter:
    """Class for visualization in the form of graphs (also saves to text file)
     the state of the robot 
     the state of the model 
     the specified trajectory (path)
     Control action (cmd_vel)
     """

    def __init__(self):
        rospy.init_node("plotter", anonymous=True)

        # if true -> show plots on shutdown
        self.show_plots = rospy.get_param("~show_plots", False)
        # if true -> save data with the start time in the name
        self.track_time = rospy.get_param("~track_time", False)
        # path to the output data folder
        self.module_path = rospy.get_param("~output_file")
        # output folder name
        self.output_folder = rospy.get_param("~output_folder")
        #
        self.timeout = int(rospy.get_param("~timeout", 0))

        self.module_path = self.module_path + self.output_folder
        if self.track_time:
            self.module_path += '/' + str(datetime.now().strftime('%Y-%m-%d-%H:%M:%S'))

        # declare tf buffer and listener for working with TF transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # node init time
        # self.init_time = round(rospy.get_time(), 5)
        self.init_time = time_.time()
        # flag for the first callback (used in Tf callback)
        self.first_tick = True
        # container for trajectory (/path) fron path_publisher node
        self.trajectory = {'x': [], 'y': []}
        # container for /cmd_vel (input control) from path_follower node
        self.control = {'t': [], 'x': [], 'yaw': []}
        # container for robot state
        self.robot_state = {'t': [], 'x': [], 'y': [], 'yaw': []}
        # container for model state
        self.model_state = {'t': [], 'x': [], 'y': [], 'yaw': []}
        # time spent running the simulator (for automated tests)
        self.time_spent = 0
        self.current_control = list()
        # declare subscribers
        self.trajectory_sub = rospy.Subscriber("/path", Path, self.path_callback)
        self.control_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Timer(rospy.Duration(0.033), self.timer_callback)

        if self.timeout > 0:
            rospy.Timer(rospy.Duration(1), self.timeout_callback)

        # set the function that will be executed when shutdown
        rospy.on_shutdown(self.on_shutdown)

    def timer_callback(self, timer_event):
        if len(self.current_control) > 0:
            self.time = time_.time() - self.init_time
            self.control['t'].append(self.time)
            self.control['x'].append(self.current_control[0])
            self.control['yaw'].append(self.current_control[1])
            self.get_states()

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

        # self.time = round(rospy.get_time() - self.init_time, 5)
        # self.time = time_.time() - self.init_time
        # self.control['t'].append(self.time)
        # self.control['x'].append(msg.linear.x)
        # self.control['yaw'].append(msg.angular.z)
        self.current_control = list([msg.linear.x, msg.angular.z])
        # self.get_states()


    def get_states(self, timer_event=None):
        """stores msg data about robot state
         and model state in separate containers"""

        # if it is the first callback set init time
        self.fill_state(dst_frame='base_link', state=self.robot_state)
        # self.fill_state(dst_frame='model_link', state=self.model_state)

    def fill_state(self, dst_frame, state={}, src_frame='odom'):
        """Receives the state of the model or robot via TF transformation"""

        try:
            tf_transform = self.tf_buffer.lookup_transform(src_frame, dst_frame, rospy.Time(),
                                                           rospy.Duration(0.2)).transform

            trans_vec = tf_transform.translation
            rot_quat = tf_transform.rotation
            yaw = euler_from_quaternion([rot_quat.x, rot_quat.y, rot_quat.z, rot_quat.w])[2]
            # self.time = round(rospy.get_time() - self.init_time, 5)
            state['t'].append(self.time)
            state['x'].append(trans_vec.x)
            state['y'].append(trans_vec.y)
            state['yaw'].append(yaw)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return tf.LookupException

    def process_collected_data(self, data, name='', plot_type='xy'):
        """Builds and saves a graph from data,
          saves data to an output file """

        plt.figure(name)
        if plot_type == 'xy':
            plot_xy_data(x=data['x'], y=data['y'])
        elif plot_type == 'xt':
            plot_xy_data(x=data['t'], y=data['x'])
        elif plot_type == 'x':
            plot_data(data['x'])
        elif plot_type == 'y':
            plot_data(data['y'])
        path = self.module_path + '/data/'
        write_to_file(path=path, data=data, file_name=name)
        path = self.module_path + '/pictures'
        save_plot(path=path, name=name)
        # all_data = [self.robot_state, self.model_state, self.trajectory]
        # self.build_general_graph(all_data, self.module_path + '/pictures')

    def build_general_graph(self, data, folder):
        """Build a graph containing information about what the trajectory was,
         how the robot (robot state) passed it and how the model (model state) passed it """

        # TODO make a universal function

        folder_path = self.module_path + folder

        x1, y1 = np.array(data[0]['x']), np.array(data[0]['y'])
        x2, y2 = np.array(data[1]['x']), np.array(data[1]['y'])
        x3, y3 = np.array(data[2]['x']), np.array(data[2]['y'])

        plt.rcParams.update({'font.size': 14})  # font size
        plt.rcParams['figure.figsize'] = (11.0, 8.0)
        plt.figure("trajectory and states")

        plt.plot(x1, y1, color='b', label='robot state', linewidth=3)
        # plt.plot(x2, y2, color='r', label='model state', linewidth=3)
        plt.plot(x3, y3, color='g', label='trajectory', linewidth=3)

        base_link_deviation = str(round(rospy.get_param("/base_link_deviation", 0), 5))
        model_deviation = str(round(rospy.get_param("/model_deviation", 0), 5))
        plt.text(x=0, y=4, s='Base_link dev = {}, Model dev = {}'.format(base_link_deviation,
                                                                         model_deviation),
                                                                         fontsize=14)

        plt.xlabel('X')
        plt.ylabel('Y')
        plt.legend(loc='best')  # or loc=1
        plt.grid(True)
        save_plot(folder_path, name='general_graph', fmt='png')

        os.makedirs(self.module_path + '/data')
        with open(self.module_path + '/data/deviation.txt', 'w+') as file:
            file.writelines('Base link dev = {}\n'.format(base_link_deviation))
            file.writelines('Model link dev = {}\n'.format(model_deviation))
       

        # plt.show()

    def on_shutdown(self):
        """ """

        # unsubscribe
        self.trajectory_sub.unregister()
        self.control_sub.unregister()

        # Process and save collected data
        data = [self.robot_state, self.model_state, self.trajectory]
        self.build_general_graph(data, '/pictures')

        self.process_collected_data(name='trajectory', data=self.trajectory)
        self.process_collected_data(name='control', data=self.control, plot_type='xt')
        self.process_collected_data(name='robot_state', data=self.robot_state)
        # self.process_collected_data(name='model_state', data=self.model_state)

        rospy.logwarn("Plotter: output data folder - {}".format(self.module_path))

        if self.show_plots:
            show_graph()


def main():
    """ """
    plotter = Plotter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.logerr("Shutting down")


if __name__ == "__main__":
    main()
