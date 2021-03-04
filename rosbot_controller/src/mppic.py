#!/usr/bin/env python3
# license removed for brevity
import time
import sys
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
from nav_msgs.msg import Path
import nnio
from tf2_msgs.msg import TFMessage
from modules.rosbot import Rosbot, RobotState, RobotControl
from modules.rosbot import Goal, euler_to_quaternion, quaternion_to_euler


class MPPIController:
    """

    """

    def __init__(self, node_name, model_path):
        self.node_name = node_name
        rospy.init_node(self.node_name, anonymous=True)

        self.cmd_topic = rospy.get_param('~cmd_topic', "/cmd_vel")
        self.parent_frame = rospy.get_param('~parent_frame', "odom")
        self.robot_frame = rospy.get_param('~robot_frame', "base_link")

        # load NN model 
        self.model = self.load_nn_model(model_path)
        self.robot = Rosbot()
        self.robot_state = RobotState()
        self.current_goal = Goal()
        self.control_vector = RobotControl(0.0, 0.0)

        self.cmd_freq = 30.0  # Hz
        self.dt = 1.0 / self.cmd_freq
        self.rate = rospy.Rate(self.cmd_freq)

        self.goal_queue = []
        self.path = []
        self.path_index = 0
        self.got_path = False

        # declare subscribers, publishers and timer
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)
        self.path_sub = rospy.Subscriber("/path", Path, self.path_callback)
        self.tf_sub = rospy.Subscriber("/tf", TFMessage, self.update_state)
        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=5)
        # rospy.Timer(rospy.Duration(self.dt), self.update_state)

    def update_state(self, msg):
        ## TODO from tf transform
        for item in msg.transforms:
            if (item.header.frame_id == self.parent_frame
                and item.child_frame_id == self.robot_frame):

                x, y = item.transform.translation.x, item.transform.translation.y
                yaw = quaternion_to_euler(item.transform.rotation)[0]
                print(x,y,yaw)
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

    def publish_control(self, control):
        """
        :param control: control vector of RobotControl type
        """
        twist_cmd = Twist()
        twist_cmd.linear.x = control.v
        twist_cmd.linear.y = 0
        twist_cmd.linear.z = 0
        twist_cmd.angular.x = 0
        twist_cmd.angular.y = 0
        twist_cmd.angular.z = control.w
        self.cmd_pub.publish(twist_cmd)

    def loss_for_goal(self):
        pass

    def predict_multi_step(self, control):
        traj = []
        return traj

    def loss_for_traj(self, traj):
        return 0

    def loss_for_control(self, control):
        traj = self.predict_multi_step(control)
        loss = self.loss_for_traj(traj)


    def run(self):
        # export model with bath_size = 100
        rollout_num = 100 # K
        timesteps_num = 10  # T
        control = np.zeros([1, timesteps_num, 2])  
        std = 0.5 # standart deviation
        while not rospy.is_shutdown():
            # update current model state
            self.robot.set_state(self.robot_state)
            for i in range(10):
                # generate some control
                control_seqs = control + np.random.normal(0, std, (rollout_num, timesteps_num, 2))
                # 
                control_seqs_loss = self.loss_for_control(control_seqs)
                
                opt_ind = np.argmin(control_seqs_loss, axis=0) # TODO change argmin

                control = control_seqs[opt_ind]

            # publish control[0]
            control = np.concatenate([control[:,1:], control[:,-1:]], axis=1)
            # 
            self.rate.sleep()


    def load_nn_model(self, model_path):
        """
        load NN model (.onnx) from given path, using nnio module
        """
        return nnio.ONNXModel(model_path)


def main():
    model_path = sys.argv[1]
    mppic = MPPIController('mppic', model_path)
    mppic.run()
    try:
        rospy.spin()
    except:
        pass


if __name__ == '__main__':
    main()
