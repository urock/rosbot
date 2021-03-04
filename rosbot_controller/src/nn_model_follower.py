#!/usr/bin/env python
# license removed for brevity
import rospy
import sys
import numpy as np
import nnio
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist, TransformStamped
from modules.rosbot import Rosbot, RobotState, RobotControl, euler_to_quaternion


class NNModelRunner:
    """

    """

    def __init__(self, node_name, model_path):
        self.node_name = node_name
        rospy.init_node(self.node_name, anonymous=True)
        self.parent_frame = rospy.get_param('~parent_frame', "odom")
        self.model_frame = rospy.get_param('~robot_frame', "nn_model_link")
        self.cmd_topic = rospy.get_param('~cmd_topic', "/cmd_vel")
        self.model_path = model_path

        self.robot = Rosbot()
        self.model_state = RobotState()

        self.cmd_freq = 30.0  # Hz
        self.dt = 1.0 / self.cmd_freq
        self.rate = rospy.Rate(self.cmd_freq)

        self.control_vector = RobotControl(0.0, 0.0)
        self.last_timestamp = rospy.Time.now().to_sec()
        self.model = None
        self.cmd_sub = rospy.Subscriber(self.cmd_topic, Twist, self.command_callback)
        self.tf_pub = rospy.Publisher('/tf', TFMessage, queue_size=1)

    def broadcast_model_tf(self, state):
        """
        Send TF transform from odom to nn_model_link
        Args:
            state: current nn model state
        Return:
            _
        """
        q = euler_to_quaternion(state.yaw, 0, 0)
        msg = TransformStamped()
        msg.header.seq = 0
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "odom"
        msg.child_frame_id = self.model_frame
        msg.transform.translation.x = state.x
        msg.transform.translation.y = state.y
        msg.transform.translation.z = 0.1
        msg.transform.rotation.x = q[0]
        msg.transform.rotation.y = q[1]
        msg.transform.rotation.z = q[2]
        msg.transform.rotation.w = q[3]

        self.tf_pub.publish(TFMessage([msg]))

    def command_callback(self, msg):
        """
        save msg to contro_vector (new current control)
        """
        self.control_vector = RobotControl(msg.linear.x, msg.angular.z)
        # rospy.logerr(self.robot_frame + ": " + self.control_vector.to_str())

    def print_state(self, state):
        """
        print current state
        """
        rospy.loginfo(state.to_str())

    def load_nn_model(self):
        """
        load NN model (.onnx) from given path, using nnio module
        """
        self.model = nnio.ONNXModel(self.model_path)

    def run(self):
        self.load_nn_model()
        while not rospy.is_shutdown():
            # current_timestamp = rospy.Time.now().to_sec()
            # dt = current_timestamp - self.last_timestamp
            self.model_state = self.robot.update_state_by_nn_model(
                self.model,
                self.control_vector,
                self.dt
            )
            # rospy.logerr("NN_MODEL: " + self.model_state.to_str())
            self.broadcast_model_tf(self.model_state)
            # self.last_timestamp = current_timestamp
            self.rate.sleep()


def main():
    model_path = sys.argv[1]
    robot_model = NNModelRunner('robot_model', model_path)
    robot_model.run()


if __name__ == '__main__':
    main()
