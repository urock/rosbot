#!/usr/bin/env python
# license removed for brevity
import rospy
import os
import tf2_ros
import tf
import numpy as np
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Path
from tf2_msgs.msg import TFMessage
import matplotlib.pyplot as plt
from geometry_msgs.msg import Pose, PoseStamped, Twist, Quaternion
from datetime import datetime
import pathlib


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

        rospy.logwarn("Show plots - {}".format(self.show_plots))
        rospy.logwarn("Track time - {}".format(self.show_plots))
        rospy.logwarn("Output data folder - {}".format(self.module_path))

        # declare tf buffer and listener for working with TF transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # 
        self.init_time = rospy.Time.now().secs
        self.first_tick = True
        # container for trajectory (/path) fron path_publisher node
        self.trajectory = {'x': [], 'y': []}
        # container for /cmd_vel (input control) from path_follower node
        self.control = {'t' : [], 'x': [], 'y': []}
        # container for robot state
        self.robot_state = {'t' : [], 'x': [], 'y': [], 'yaw': []}
        # container for model state
        self.model_state = {'t' : [], 'x': [], 'y': [], 'yaw': []}

        # declare subscribers
        self.trajectory_sub = rospy.Subscriber("/path", Path, self.path_callback)
        self.control_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.tf_sub = rospy.Subscriber('/tf', TFMessage, self.tf_callback)

        # set the function that will be executed when shutdown
        rospy.on_shutdown(self.on_shutdown)

    def path_callback(self, msg):
        """stores path messages in a separate container"""

        for p in msg.poses:
            x, y = p.pose.position.x, p.pose.position.y
            self.trajectory['x'].append(x)
            self.trajectory['y'].append(y)

    def cmd_vel_callback(self, msg):
        """stores control messages in a separate container"""

        time = rospy.Time.now().secs - self.init_time
        self.control['t'].append(time)
        self.control['x'].append(msg.linear.x)
        self.control['y'].append(msg.linear.y)

    def tf_callback(self, msg):
        """stores msg data about robot state
         and model state in separate containers"""

        if self.first_tick:
            self.first_tick = False
            self.init_time = rospy.Time.now().secs

        self.fill_state(dst_frame='base_link', state=self.robot_state)
        self.fill_state(dst_frame='model_link', state=self.model_state)

    def fill_state(self, dst_frame, state={}, src_frame='odom'):
        """Receives the state of the model or robot via TF transformation"""

        try:
            tf_transform = self.tf_buffer.lookup_transform(src_frame, dst_frame, rospy.Time(),
                                                           rospy.Duration(0.2)).transform

            trans_vec = tf_transform.translation
            rot_quat = tf_transform.rotation
            yaw = euler_from_quaternion([rot_quat.x, rot_quat.y, rot_quat.z, rot_quat.w])[2]
            time = rospy.Time.now().secs - self.init_time
            state['t'].append(time)
            state['x'].append(trans_vec.x)
            state['y'].append(trans_vec.y)
            state['yaw'].append(yaw)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return tf.LookupException

    def write_to_file(self, data, file_name):
        """Saves data to the output file"""

        pwd = os.getcwd()
        # os.chdir(os.path.expanduser('~'))
        path = self.module_path + '/data/'
        # print(path)
        os.chdir(os.path.expanduser('~'))
        if not os.path.exists(path):
            os.makedirs(path)

        output_file = open(path + file_name + '.txt', 'w')

        for i in range(0, len(data.values()[0])):
            item = str()
            for key in ('t', 'x', 'y', 'yaw'):
                if key in data.keys():
                    item = item + str(round(data[key][i], 2)) + ' '
            output_file.write(str(item) + '\n')

        output_file.close()
        os.chdir(pwd)

    def plot_xy_data(self, data):
        """Build a graph from x and y"""

        x = np.array(data['x'])
        y = np.array(data['y'])
        plt.plot(x, y)
        plt.grid(True)

    def plot_data(self, data):
        """Build a graph from x or y """

        plt.plot(data)
        plt.grid(True)

    def save_plot(self, name='', fmt='png'):
        """Saves graph to the output pkg"""

        pwd = os.getcwd()
        path = self.module_path + '/pictures'
        # print(path)
        os.chdir(os.path.expanduser('~'))
        if not os.path.exists(path):
            os.makedirs(path)
        os.chdir(path)
        plt.savefig('{}.{}'.format(name, fmt), fmt='png')
        os.chdir(pwd)

    def show_graph(self):
        """ """

        try:
            plt.show()
            plt.close()
        except:
            pass

    def process_collected_data(self, data, name='', plot_type='xy'):
        """Builds and saves a graph from data,
          saves data to an output file """

        if self.track_time:
            name = name + "_" + datetime.now().strftime('%Y-%m-%d %H:%M:%S')

        plt.figure(name)
        if plot_type == 'xy':
            self.plot_xy_data(data)
        elif plot_type == 'x':
            self.plot_data(data['x'])
        elif plot_type == 'y':
            self.plot_data(data['y'])
        self.write_to_file(data, file_name=name)
        self.save_plot(name=name)
        # self.show_graph()

    def on_shutdown(self):
        """ """

        # unsubscribe
        self.trajectory_sub.unregister()
        self.control_sub.unregister()
        self.tf_sub.unregister()

        # Process and save collected data 
        self.process_collected_data(name='trajectory', data=self.trajectory)
        self.process_collected_data(name='control', data=self.control, plot_type='x')
        self.process_collected_data(name='robot_state', data=self.robot_state)
        self.process_collected_data(name='model_state', data=self.model_state)

        if self.show_plots:
            # show all graphs
            plt.show()


def main():
    """ """
    plotter = Plotter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.logerr("Shutting down")


if __name__ == "__main__":
    main()
