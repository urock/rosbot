#!/usr/bin/env python
# license removed for brevity
import rospy
# import tf
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist
import nnio
from modules.rosbot import Rosbot, RobotState, RobotControl

j = 1

def euler_to_quaternion(yaw, pitch, roll):
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(
        pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(
        pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(
        pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(
        pitch / 2) * np.sin(yaw / 2)

    return [qx, qy, qz, qw]


class NNModelRunner:
    """

    """

    def __init__(self, node_name):
        self.node_name = node_name
        rospy.init_node(self.node_name, anonymous=True)
        self.robot_frame = rospy.get_param('~robot_frame', "NNMODEL")
        self.cmd_topic = rospy.get_param('~cmd_topic', "/cmd_vel")
        self.model_path = rospy.get_param('~model_path',
                                          "/home/vytautas/MS/catkin_ws/src/rosbot/rosbot_controller/src/model.onnx")
        self.odom_frame = 'odom'

        self.robot = Rosbot()
        self.model_state = RobotState()

        # self.tf_br = tf.TransformBroadcaster()

        self.cmd_freq = 30.0  # Hz
        self.dt = 1.0 / self.cmd_freq
        self.rate = rospy.Rate(self.cmd_freq)

        self.control_vector = RobotControl(0.0, 0.0)
        self.cmd_sub = rospy.Subscriber(self.cmd_topic, Twist, self.command_callback)
        self.path_pub = rospy.Publisher('/NN_MODEL', Marker, queue_size=1)
        self.last_timestamp = rospy.Time.now().to_sec()
        self.model = None
        self.model_states = list()
        self.model_state_set = set()

    # def broadcast_model_tf(self, state):
    #     pose = (state.x, state.y, 0.0)
    #     orient = tf.transformations.quaternion_from_euler(0, 0, state.yaw)
    #
    #     self.tf_br.sendTransform(pose, orient,
    #                              rospy.Time.now(),
    #                              self.robot_frame,
    #                              self.odom_frame)

    def publish_arrow(self, point):
        pMarker = Marker()
        pMarker.header.frame_id = 'odom'
        pMarker.header.stamp = rospy.Time.now()
        pMarker.id = 0
        pMarker.type = Marker.ARROW
        pMarker.action = Marker.ADD  # ADD
        pMarker.pose.position.x = point.x
        pMarker.pose.position.y = point.y
        pMarker.pose.position.z = 0.1
        orinet = euler_to_quaternion(point.yaw, 0, 0)
        # [qx, qy, qz, qw]
        pMarker.pose.orientation.x = orinet[0]
        pMarker.pose.orientation.y = orinet[1]
        pMarker.pose.orientation.z = orinet[2]
        pMarker.pose.orientation.w = orinet[3]
        pMarker.scale.x = 0.3
        pMarker.scale.y = 0.05
        pMarker.scale.z = 0.1

        pMarker.color.r = 1.0
        pMarker.color.g = 1.0
        pMarker.color.b = 0.0
        pMarker.color.a = 1.0
        self.path_pub.publish(pMarker)

    def publish_sphere(self, point, i):
        print(i)
        pMarker = Marker()
        pMarker.header.frame_id = 'odom'
        pMarker.header.stamp = rospy.Time.now()
        pMarker.id = i
        pMarker.lifetime = rospy.Duration(0)
        pMarker.type = Marker.SPHERE
        pMarker.action = Marker.ADD  # ADD
        pMarker.pose.position.x = point.x
        pMarker.pose.position.y = point.y
        pMarker.pose.position.z = 0.1
        orinet = euler_to_quaternion(point.yaw, 0, 0)
        # [qx, qy, qz, qw]
        pMarker.pose.orientation.x = orinet[0]
        pMarker.pose.orientation.y = orinet[1]
        pMarker.pose.orientation.z = orinet[2]
        pMarker.pose.orientation.w = orinet[3]
        pMarker.scale.x = 0.02
        pMarker.scale.y = 0.02
        pMarker.scale.z = 0.02

        pMarker.color.r = 1.0
        pMarker.color.g = 1.0
        pMarker.color.b = 0.0
        pMarker.color.a = 1.0
        pMarker.lifetime = rospy.Duration(0)
        # pMarker.points.append(pMarker.pose.position)
        self.path_pub.publish(pMarker)



    def command_callback(self, msg):
        self.control_vector = RobotControl(msg.linear.x, msg.angular.z)
        # rospy.logerr(self.robot_frame + ": " + self.control_vector.to_str())

    def print_state(self, state):
        rospy.loginfo(state.to_str())

    def load_nn_model(self):
        self.model = nnio.ONNXModel("model.onnx")
        # x = np.array([[2.1497, 0.0225, 0.0735, -1.3472, -2.2071]], dtype=np.float32)
        # onnx_out = self.model(x)
        # print("onnx out {}".format(onnx_out))

    def run(self):
        self.load_nn_model()
        while not rospy.is_shutdown():
            current_timestamp = rospy.Time.now().to_sec()
            dt = current_timestamp - self.last_timestamp
            # rospy.logerr("CONTROL" + self.control_vector.to_str())
            self.model_state = self.robot.update_state_by_nn_model(self.model, self.control_vector,
                                                                   dt)
            rospy.logerr("NN_MODEL: " + self.model_state.to_str())
            self.model_states.append(self.model_state)
            self.model_state_set.add(self.model_state)
            self.publish_arrow(self.model_state)
            self.publish_sphere(self.model_state, i=len(self.model_states))
            # print(self.model_states[0])
            self.last_timestamp = current_timestamp

            self.rate.sleep()


def main():
    robot_model = NNModelRunner('robot_model')
    robot_model.run()


if __name__ == '__main__':
    main()
