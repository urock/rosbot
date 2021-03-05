#!/usr/bin/env python3
# license removed for brevity
import time
import sys
import os
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
from nav_msgs.msg import Path
import nnio
from tf2_msgs.msg import TFMessage
from modules.rosbot import Rosbot, RobotState, RobotControl
from modules.rosbot import Goal, quaternion_to_euler


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
        self.stop_mppi = False

        # declare subscribers, publishers and timer
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)
        self.path_sub = rospy.Subscriber("/path", Path, self.path_callback)
        self.tf_sub = rospy.Subscriber("/tf", TFMessage, self.update_state)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)

    def update_state(self, msg):
        ## TODO from tf transform
        for item in msg.transforms:
            if (item.header.frame_id == self.parent_frame
                and item.child_frame_id == self.robot_frame):

                x, y = item.transform.translation.x, item.transform.translation.y
                yaw = quaternion_to_euler(item.transform.rotation)[0]
                self.robot_state = RobotState(x, y, yaw)
                self.robot.set_state(self.robot_state)

                if self.robot.goal_reached(self.current_goal):
                    if self.goal_queue:
                        # next goal
                        self.current_goal = self.goal_queue.pop(0)
                        self.path_index += 1
                        rospy.logerr(self.robot_frame + ": new current_goal = " + self.current_goal.to_str())
                    else:
                        # end of trajectory
                        self.publish_control(RobotControl())
                        self.stop_mppi = True

    def goal_callback(self, msg):
        """

        """
        x, y = msg.pose.position.x, msg.pose.position.y
        self.goal_queue.append(Goal(x, y))
        rospy.logwarn("added goal to queue: x -> {:.2f}, y -> {:.2f}".format(x, y))

    def path_callback(self, msg):
        """
        :msg : 
        """
        for p in msg.poses:
            x, y = p.pose.position.x, p.pose.position.y
            self.goal_queue.append(Goal(x, y))
            self.path.append(Goal(x, y))
        self.got_path = True

    def wait_for_path(self):
        """

        """
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

  
    def loss_for_traj(self, trajectories, goal):
        """

        """
        traj_x, traj_y = trajectories[:,:,0], trajectories[:,:,1]
        loss_x = traj_x - goal.x
        loss_y = traj_y - goal.y
        loss = np.sqrt(loss_x ** 2 + loss_y**2)
        loss = loss * np.linspace(1, 1.1, loss.shape[1])[None]   # ??
        # loss = loss.sum(1)
        loss = loss[:,-1]
        return loss
    

    def predict_trajectories(self, velocities):
        '''
        inputs:
        velocities
        outputs:
        '''
        v, w = velocities[:,:,0], velocities[:,:,1] 
        current_yaw = self.robot_state.yaw
        yaw = current_yaw + np.cumsum(w * self.dt, axis=1)       
        vx = v * np.cos(yaw)
        vy = v * np.sin(yaw)
        x = np.cumsum(vx * self.dt, axis=1)
        y = np.cumsum(vy * self.dt, axis=1)
        result_xya= np.concatenate([
            x[:, :, None],
            y[:, :, None],
            yaw[:, :, None],
        ], axis=2)
        return result_xya 


    # TODO ASK first element (0 0) ? am I dummy? 
    def predict_multi_step(self, batch_x):
        '''
        inputs:
        outputs:
        '''
        
        state = batch_x[:, 0] 
        batch_y = [batch_x[:, :1, :2]]          # v and w
        for t in range(1, batch_x.shape[1] + 1):
            state = np.array(state, dtype=np.float32)
            # print(state.shape)
            pred_vw = self.model(state)
            if t != batch_x.shape[1]:
                state = np.concatenate([pred_vw, batch_x[:, t, 2:]], axis=1)
            batch_y.append(pred_vw[:, None])    # add axis for time
        
        #concatenate along time axis
        batch_y = np.concatenate(batch_y, axis=1)
        return batch_y
        

    def loss_for_control(self, control_seqs, goal):
        """
        Calcuate loss for given control seqence
        """

        shape = control_seqs.shape
        init_state = np.zeros((shape[0], shape[1], 5))
        init_state[:,0,0] = self.robot.v
        init_state[:,0,1] = self.robot.w
        init_state[:,:,2:4] = control_seqs
        init_state[:,:,4] = self.dt

        predicted_velocities = self.predict_multi_step(init_state)
        trajectories = self.predict_trajectories(predicted_velocities)
        loss_for_control = self.loss_for_traj(trajectories, goal)
        return loss_for_control
        

    def run(self):
        """
        
        """
        # export model with bath_size = 100
        rollout_num = 100              # K 100  (batch size)
        timesteps_num = 10             # T 10
        control = np.zeros([timesteps_num, 2])  
        std = 0.02 # standart deviation
        self.current_goal.x = 2 # TEST
        self.current_goal.y = 2 # TEST
        while not rospy.is_shutdown():
            # TODO if we have reached the goal, select the next goal
            rospy.loginfo("Robot state = {}".format(self.robot_state.to_str()))
            rospy.loginfo("Current goal = {}".format(self.current_goal.to_str()))
            rospy.loginfo("Stop sim")
            os.system('rosservice call /gazebo/pause_physics') # stop sim
            time_start = time.time()
            for i in range(100):
                # generate some control
                control_seqs = control[None] + np.random.normal(0, std, size=(rollout_num, timesteps_num, 2))
                control_seqs = np.clip(control_seqs, -1, 1)
                control_seqs_loss = self.loss_for_control(control_seqs, self.current_goal)
                opt_ind = np.argmin(control_seqs_loss, axis=0) # TODO change argmin
                control = control_seqs[opt_ind]
                
            print(control)
            self.publish_control(RobotControl(control[0][0], control[0][1]))
            rospy.loginfo("Continue sim")
            os.system('rosservice call /gazebo/unpause_physics') # start sim
            control = np.concatenate([control[:,1:], control[:,-1:]], axis=1)
            rospy.logwarn( "Execution time is = {} sec".format(time.time() - time_start)) # 0.01
            self.rate.sleep()
        # rospy.logerr("STOP MPPIC")

    def load_nn_model(self, model_path):
        """
        load NN model (.onnx) from given path, using nnio module
        """
        return nnio.ONNXModel(model_path)


def main():
    model_path = sys.argv[1]
    mppic = MPPIController('mppic', model_path)
    # mppic.wait_for_path()
    mppic.run()
    try:
        rospy.spin()
    except:
        pass


if __name__ == '__main__':
    main()
