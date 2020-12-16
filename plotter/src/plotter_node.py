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


class Plotter:
    """ """

    def __init__(self):
        rospy.init_node("plotter", anonymous=True)

        # declare tf buffer and listener for working with TF transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.module_path = "./src/rosbot/plotter"
        # container for /path (trajectory) fron path_publisher node
        self.trajectory = {'x': [], 'y': []}  # dict {'x':[x0,x1,x2,...xn], 'y':[y0,y1,y2,...yn]}
        # container for /cmd_vel (input control) from path_follower node
        self.control = {'x': [], 'y': []}
        # container for robot state
        self.robot_state = {'x': [], 'y': [], 'yaw': []}
        # container for model state
        self.model_state = {'x': [], 'y': [], 'yaw': []}

        # declare subscribers
        self.trajectory_sub = rospy.Subscriber("/path", Path, self.path_callback)
        self.control_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.tf_sub = rospy.Subscriber('/tf', TFMessage, self.tf_callback)

        # set the function that will be executed when shutdown
        rospy.on_shutdown(self.on_shutdown)

    def path_callback(self, msg):
        """ """

        for p in msg.poses:
            x, y = p.pose.position.x, p.pose.position.y
            self.trajectory['x'].append(x)
            self.trajectory['y'].append(y)

    def cmd_vel_callback(self, msg):
        """ """

        self.control['x'].append(msg.linear.x)
        self.control['y'].append(msg.linear.y)

    def tf_callback(self, msg):
        """ """

        self.fill_state(dst_frame='base_link', state=self.robot_state)
        self.fill_state(dst_frame='model_link', state=self.model_state)

    def fill_state(self, dst_frame, state={}, src_frame='odom'):
        """ """

        try:
            tf_transform = self.tf_buffer.lookup_transform(src_frame, dst_frame, rospy.Time(),
                                                           rospy.Duration(0.2)).transform

            trans_vec = tf_transform.translation
            rot_quat = tf_transform.rotation
            yaw = euler_from_quaternion([rot_quat.x, rot_quat.y, rot_quat.z, rot_quat.w])[2]
            state['x'].append(trans_vec.x)
            state['y'].append(trans_vec.y)
            state['yaw'].append(yaw)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return tf.LookupException

    def write_to_file(self, data, file_name):
        """ """

        pwd = os.getcwd()
        path = self.module_path + '/data'
        if not os.path.exists(path):
            os.mkdir(path)
        os.chdir(path)
        output_file = open(file_name + '.txt', 'w')

        for i in range(0, len(data.values()[0])):
            item = str()
            for key in ('x', 'y', 'yaw'):
                if key in data.keys():
                    item = item + str(round(data[key][i], 2)) + ' '
            output_file.write(str(item) + '\n')

        output_file.close()
        os.chdir(pwd)

    def plot_xy_data(self, data):
        """   """

        x = np.array(data['x'])
        y = np.array(data['y'])
        plt.plot(x, y)
        plt.grid(True)

    def save_plot(self, name='', fmt='png'):
        """ """

        pwd = os.getcwd()
        path = self.module_path + '/pictures'
        if not os.path.exists(path):
            os.mkdir(path)
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

    def process_collected_data(self, name, data):
        """ """

        plt.figure(name)
        self.plot_xy_data(data)
        self.write_to_file(data, file_name=name)
        self.save_plot(name=name)
        self.show_graph()

    def on_shutdown(self):
        """ """

        self.trajectory_sub.unregister()
        self.control_sub.unregister()
        self.tf_sub.unregister()

        self.process_collected_data(name='trajectory', data=self.trajectory)
        self.process_collected_data(name='control', data=self.control)
        self.process_collected_data(name='robot_state', data=self.robot_state)
        self.process_collected_data(name='model_state', data=self.model_state)


def main():
    plotter = Plotter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.logerr("Shutting down")


if __name__ == "__main__":
    main()
