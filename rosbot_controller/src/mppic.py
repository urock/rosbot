#!/usr/bin/env python3
# license removed for brevity
import time
import sys
import os
import math
import numpy as np
import nnio
import matplotlib.pyplot as plt
import rospy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped, Twist, Pose, Point
from geometry_msgs.msg import Vector3, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from modules.rosbot import Rosbot, RobotState, RobotControl
from modules.rosbot import Goal, quaternion_to_euler

class MPPIController:
    """

    """

    def __init__(self, node_name, model_path):
        rospy.init_node(node_name, anonymous=True)

        # get parameters
        self.cmd_topic = rospy.get_param('~cmd_topic', "/cmd_vel")
        self.parent_frame = rospy.get_param('~parent_frame', "odom")
        self.robot_frame = rospy.get_param('~robot_frame', "base_link")
        self.cmd_freq = int(rospy.get_param('~cmd_freq', 30)) # Hz

        # load NN model 
        self.model = self.load_nn_model(model_path)
        # declare robot and current and previous state
        self.robot = Rosbot()
        self.robot_state = RobotState()
        self.prev_state = RobotState()
        self.current_goal = Goal()

        self.dt = 1.0 / self.cmd_freq
        self.rate = rospy.Rate(self.cmd_freq)

        self.goal_queue = []
        self.got_path = False
        self.last_tf_callback_time = None

        # declare subscribers
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)
        self.path_sub = rospy.Subscriber("/path", Path, self.path_callback)
        self.tf_sub = rospy.Subscriber("/tf", TFMessage, self.update_state)
        # declare publishers
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
        self.marker_pub = rospy.Publisher('/all_trajectories', MarkerArray, queue_size=10)
        self.curr_path_pub = rospy.Publisher("/curr_path", Path, queue_size=5)
        

    def update_state(self, msg):
        """
        """
        ## TODO from tf transform
        for item in msg.transforms:
            if self.last_tf_callback_time is None:
                self.last_tf_callback_time = time.time()
                break
            if (item.header.frame_id == self.parent_frame
                and item.child_frame_id == self.robot_frame):
                # time delta calculation
                dt = time.time() - self.last_tf_callback_time   
                # update previous state
                self.prev_state = self.robot_state              
                # update robot state
                x, y = item.transform.translation.x, item.transform.translation.y
                yaw = quaternion_to_euler(item.transform.rotation)[0]
                self.robot_state = RobotState(x, y, yaw)    
                # calculate robot lin velocity
                vx = (self.robot_state.x - self.prev_state.x) / dt
                vy = (self.robot_state.y - self.prev_state.y) / dt
                v = math.sqrt(vx**2 + vy**2)
                alpha = math.atan2(vy, vx)
                v = v * math.cos(alpha - self.robot_state.yaw)
                # calculate robot ang velocity
                d_yaw = (self.robot_state.yaw - self.prev_state.yaw)
                d_yaw = (d_yaw + math.pi) % (2 * math.pi) - math.pi
                w = d_yaw / dt
                # update robot velocities
                self.robot.v = v
                self.robot.w = w
                # update last tf callback time
                self.last_tf_callback_time = time.time()
                # check if goal reached
                self.robot.set_state(self.robot_state)
                if self.robot.goal_reached(self.current_goal):
                    if self.goal_queue:
                        # next goal
                        self.current_goal = self.goal_queue.pop(0)
                        rospy.logerr(self.robot_frame + ": new current_goal = " + self.current_goal.to_str())
                    else:
                        # end of trajectory
                        self.publish_control(RobotControl())

    def goal_callback(self, msg):
        """
        Add msg to the goal queue
        Args:
            msg (PoseStamped): goal msg
        """
        x, y = msg.pose.position.x, msg.pose.position.y
        yaw = quaternion_to_euler(msg.pose.orientation)[0]
        self.goal_queue.append(Goal(x, y, yaw))
        rospy.logwarn("added goal to queue: x -> {:.2f}, y -> {:.2f}".format(x, y))

    def path_callback(self, msg):
        """
        Add all point from path msg to the goal queue
        Args:
            msg (Path): path msg
        """
        for p in msg.poses:
            x, y = p.pose.position.x, p.pose.position.y
            yaw = quaternion_to_euler(p.pose.orientation)[0]
            self.goal_queue.append(Goal(x, y, yaw))
        self.got_path = True

    def wait_for_path(self):
        """
        Waits until a message with dtype = Path arrives
        """
        while not rospy.is_shutdown():
            if self.got_path:
                break
            self.rate.sleep()

    def publish_control(self, control):
        """
        Publishes controls for the rosbot
        Args:
            control: control vector of RobotControl type
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
        traj_x, traj_y, traj_yaw = trajectories[:,:, 0], trajectories[:,:, 1], trajectories[:,:, 2]
        loss_x = traj_x - goal.x
        loss_y = traj_y - goal.y
        loss_yaw = traj_yaw - goal.yaw
        # print(traj_yaw.shape, goal.yaw)
        loss = np.sqrt(loss_x ** 2 + loss_y ** 2 + loss_yaw ** 2)
        # loss = loss * np.linspace(1, 1.1, loss.shape[1])[None]   # ??
        # loss = loss.sum(1)
        # loss = loss[:,-1]
        loss = loss.min(axis=1)
        return loss
    

    def predict_trajectories(self, velocities):
        '''
        Integrates velocities according to 
        the robot model to obtain trajectories
        Args:
            velocities:
        Return
            result_xya:
        '''
        v, w = velocities[:,:,0], velocities[:,:,1] 
        current_yaw = self.robot_state.yaw
        yaw = np.cumsum(w * self.dt, axis=1)
        yaw += current_yaw - yaw[:,:1] 
        vx = v * np.cos(yaw)
        vy = v * np.sin(yaw)
        x = np.cumsum(vx * self.dt, axis=1)
        y = np.cumsum(vy * self.dt, axis=1)
        x += self.robot_state.x - x[:,:1]
        y += self.robot_state.y - y[:,:1]
        
        result_xya= np.concatenate([
            x[:, :, None],
            y[:, :, None],
            yaw[:, :, None],
        ], axis=2)
        return result_xya 


    # TODO ASK first element (0 0) ? am I dummy? 
    def predict_multi_step(self, batch_x):
        '''
        Predicts next speed
        Args:
            batch_x: 
        Return:
            batch_y:
        '''
        
        state = batch_x[:, 0] 
        batch_y = [batch_x[:, :1, :2]]          # v and w
        time_start = time.time()
        for t in range(1, batch_x.shape[1] + 1):
            state = np.array(state, dtype=np.float32)
            # print(state.shape)
            pred_vw = self.model(state)
            if t != batch_x.shape[1]:
                state = np.concatenate([pred_vw, batch_x[:, t, 2:]], axis=1)
            batch_y.append(pred_vw[:, None])    # add axis for time
        # print("predict_multi_step execution time = {}".format(time.time() - time_start))
        #concatenate along time axis
        batch_y = np.concatenate(batch_y, axis=1)
        return batch_y
        

    def loss_for_control(self, control_seqs, goal):
        """
        Calcuate loss for given control seqence
        Args:
            control_seqs:
            goal:
        Return:
            loss_for_control
            trajectories
        """
        time_start = time.time()
        shape = control_seqs.shape
        init_state = np.zeros((shape[0], shape[1], 5))
        # print(self.robot.v, self.robot.w)
        init_state[:,0,0] = self.robot.v
        init_state[:,0,1] = self.robot.w
        init_state[:,:,2:4] = control_seqs
        init_state[:,:,4] = self.dt

        predicted_velocities = self.predict_multi_step(init_state)
        trajectories = self.predict_trajectories(predicted_velocities)
        loss_for_control = self.loss_for_traj(trajectories, goal)
        # print("loss for control execution time = {}".format(time.time() - time_start))
        return loss_for_control, trajectories
        
    def visualize_trajectories(self, trajectories):
        """
        Publishes trajectories as arrays of marker points
        for visualization in Rviz
        Args:
            trajectories
        """
        marker_array = MarkerArray()
        i = 0
        for traj in trajectories:
            step = int(len(traj) * 0.1)
            for p in traj[::step]:
                marker = Marker()
                marker.id = i
                marker.header.stamp = rospy.Time.now()
                marker.header.frame_id = "odom"
                marker.lifetime = rospy.Duration(0)
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.scale = Vector3(0.01, 0.01, 0.01)
                marker.color.r, marker.color.g, marker.color.a = (0.0, 1.0, 1.0)
                marker.pose.position.x = p[0]
                marker.pose.position.y = p[1]
                marker.pose.position.z = 0.05
                marker.pose.orientation.w = 0
                i = i +1
                marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array) 


    def show_curr_path(self, traj):
        """
        Publishes a path as a message with
        the 'Path' data type for visualization in Rviz
        Args:
             traj (np.array shape = [timesteps_num, 2]): trajectory
        """
        path = Path()
        path.header.frame_id = self.parent_frame
        tx, ty = traj[:, 0], traj[:, 1]

        for i in range(len(tx)):
            path.poses.append(PoseStamped(pose=Pose(position=Point(x=tx[i], y=ty[i], z=0))))

        self.curr_path_pub.publish(path)

    def run(self):
        """
        
        """
        # export model with bath_size = 100
        rollout_num = 100                # K 100  (batch size)
        timesteps_num = 50               # T 10
        std = 0.1 # standart deviation
        control = np.asarray([[0.0, 0.0]] * timesteps_num) # control shape = [timesteps_num, 2]
        # self.current_goal.x = 2.0
        # self.current_goal.y = 0.0
        try:
            while not rospy.is_shutdown():
                rospy.loginfo("Robot state = {}".format(self.robot_state.to_str()))
                rospy.loginfo("Current goal = {}".format(self.current_goal.to_str()))
                rospy.loginfo("Stop sim")
                # os.system('rosservice call /gazebo/pause_physics') # stop sim
            
                opt_loss = []

                num_iterations = 0
                time_start = time.time()
                # control = np.asarray([[0.2, 0.0]] * timesteps_num) # control shape = [timesteps_num, 2]

                for i in range(50):
                    num_iterations += 1
                    # control[None] shape = [1, timesteps_num, 2]
                    control_seqs = control[None] + np.random.normal(0.0, std, size=(rollout_num, timesteps_num, 2))
                    # control shape = [rollout_num, timesteps_num, 2]
                    control_seqs = np.clip(control_seqs, -0.1, 0.1)
                    control_seqs_loss, trajectories = self.loss_for_control(control_seqs, self.current_goal)
                    # self.visualize_trajectories(trajectories)  # visualize all trajectories
                    opt_loss.append(np.min(control_seqs_loss)) # 
                    opt_ind = np.argmin(control_seqs_loss, axis=0)
                    # self.show_curr_path(trajectories[opt_ind]) # visualize optimal trajectory
                    control = control_seqs[opt_ind] # control shape = [timesteps_num, 2]
                    if opt_loss[-1] <= 0.05:
                        break
                execution_time = time.time() - time_start
                # plt.plot(range(len(opt_loss)), opt_loss)
                # plt.show()   
                # print(opt_loss)
                # print(control)
                # print("num_iterations = {}".format(num_iterations))
                self.publish_control(RobotControl(control[0][0], control[0][1]))
                rospy.loginfo("Continue sim")
                # os.system('rosservice call /gazebo/unpause_physics') # start sim
                control = np.concatenate([control[1:], control[-1:]], axis=0)
                rospy.logwarn( "Execution time is = {} sec".format(execution_time)) # 0.01
                self.rate.sleep()
        except KeyboardInterrupt:
            print('finish')

    def load_nn_model(self, model_path):
        """
        load NN model (.onnx) from given path, using nnio module
        """
        return nnio.ONNXModel(model_path)


def main():
    model_path = sys.argv[1]
    mppic = MPPIController('mppic', model_path)
    mppic.wait_for_path()
    mppic.run()
    try:
        rospy.spin()
    except:
        pass


if __name__ == '__main__':
    main()
