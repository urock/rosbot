#!/usr/bin/env python3

import time
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

from rosbot_controller.rosbot import Rosbot, RobotState, RobotControl
from rosbot_controller.rosbot import Goal, quaternion_to_euler

class MPPIController:
    def __init__(self, node_name):
        rospy.init_node(node_name, anonymous=True)

        # get parameters
        self.model_path = rospy.get_param('~model_path', None)
        self.cmd_topic = rospy.get_param('~cmd_topic', "/cmd_vel")
        self.parent_frame = rospy.get_param('~parent_frame', "odom")
        self.robot_frame = rospy.get_param('~robot_frame', "base_link")
        self.cmd_freq = int(rospy.get_param('~cmd_freq', 30))  # Hz

        # TODO rosparams
        self.timesteps_num = 50  
        self.batch_size = 100  

        self.curr_control = np.zeros(shape = (self.batch_size, self.timesteps_num, 2))  

        # load NN model 
        self.model = self.load_nn_model(self.model_path)
        # declare robot and current and previous state
        self.robot = Rosbot()
        self.curr_state = RobotState()
        self.prev_state = RobotState()


        self.curr_goal = Goal() 
        self.next_goal = None 

        self.dt = 1.0 / self.cmd_freq
        self.rate = rospy.Rate(self.cmd_freq)

        self.goal_queue = []
        self.got_path = False
        self.last_tf_callback_time = None
        self.stop = False

        # declare subscribers
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)
        self.path_sub = rospy.Subscriber("/path", Path, self.path_callback)
        self.tf_sub = rospy.Subscriber("/tf", TFMessage, self.update_state)
        
        # declare publishers
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
        self.marker_pub = rospy.Publisher('/all_trajectories', MarkerArray, queue_size=10)
        self.curr_path_pub = rospy.Publisher("/curr_path", Path, queue_size=5)

    def start(self):
        iter_count = 50
        limit_v = 0.5

        v_std = 0.1  # standart deviation
        w_std = 0.2  # standart deviation

        try:
            while not rospy.is_shutdown() and not self.stop:
                self.run(limit_v, v_std, w_std, iter_count)
        except KeyboardInterrupt:
            rospy.loginfo("Interrupted")


    def run(self, limit_v, v_std, w_std, iter_count):
        start = time.perf_counter()

        control = self.get_best_control(iter_count,  limit_v,  v_std, w_std)
        t = time.perf_counter() - start

        steps_passed = round( (t / self.dt) )
        steps_passed = min(steps_passed, self.timesteps_num-1)

        v_best = control[0 + steps_passed, 0]
        w_best = control[0 + steps_passed, 1]

        self.publish_control(RobotControl(v_best, w_best))

        rospy.loginfo("Take {} step of MPPI \n v = {} \n w = {}".format(steps_passed, v_best, w_best))
        rospy.loginfo("Execution time is = {:10.5f} sec".format(t))


    def get_best_control(self, iter_count, limit_v, v_std, w_std):
        best_control = self.curr_control
        for _ in range(iter_count):
            control_seqs = best_control + self.generate_noise(v_std, w_std)
            control_seqs = np.clip(control_seqs, -limit_v, limit_v) # Clip both v and w ?

            init_states = self.create_init_state(control_seqs)
            predicted_velocities = self.predict_velocities(init_states)
            trajectories = self.predict_trajectories(predicted_velocities)
            curr_losses = self.calc_losses(trajectories)

            best_ind = np.argmin(curr_losses, axis=0)
            best_loss = curr_losses[best_ind]
            best_control = control_seqs[best_ind]

            if best_loss <= 0.05:
                break

        return best_control


    def generate_noise(self, v_std, w_std):
        v_noise = np.random.normal(0.0, v_std, size=(self.batch_size, self.timesteps_num, 1))
        w_noise = np.random.normal(0.0, w_std, size=(self.batch_size, self.timesteps_num, 1))
        return np.concatenate([v_noise, w_noise], axis=2)


    def create_init_state(self, control_seqs):
        shape = control_seqs.shape
        init_states = np.zeros( (shape[0], shape[1], 5) )

        init_states[:, 0, 0] = self.robot.v
        init_states[:, 0, 1] = self.robot.w
        init_states[:, :, 2:4] = control_seqs
        init_states[:, :, 4] = self.dt

        return init_states

    def predict_velocities(self, init_states):
        """
        Filling initial states with predicted velocities along time horizont for all batches
        Args:
            [in] init_states: np.array of shape [batch, time_steps, state_dim + control_dim + 1] where 1 is for dt 
        Return:
            Predicted velocities over time horizont: np.array of shape [batch, time_steps, control_dim]
        """
        filled_states = init_states

        time_steps = filled_states.shape[1]
        for t_step in range(time_steps - 1):
            curr_batch = filled_states[:, t_step].astype(np.float32)
            curr_predicted = self.model(curr_batch) 
            filled_states[:, t_step + 1, :2] = curr_predicted 

        return filled_states[:,:,:2]


    def predict_trajectories(self, velocities):
        """
        Integrates velocities according to 
        the robot model to obtain trajectories
        Args:
            velocities:
        Return
            result_xya:
        """
        v, w = velocities[:, :, 0], velocities[:, :, 1]
        current_yaw = self.curr_state.yaw
        yaw = np.cumsum(w * self.dt, axis=1)
        yaw += current_yaw - yaw[:, :1]
        vx = v * np.cos(yaw)
        vy = v * np.sin(yaw)
        x = np.cumsum(vx * self.dt, axis=1)
        y = np.cumsum(vy * self.dt, axis=1)
        x += self.curr_state.x - x[:, :1]
        y += self.curr_state.y - y[:, :1]

        result_xya = np.concatenate([
            x[:, :, np.newaxis],
            y[:, :, np.newaxis],
            yaw[:, :, np.newaxis],
        ], axis=2)
        return result_xya

    def calc_losses(self, trajectories):
        """
        Calculate cost function (loss) for trajectories
        """
        goal = self.curr_goal
        x, y = trajectories[:, :, 0], trajectories[:, :, 1]

        d_x = x - goal.x
        d_y = y - goal.y

        loss =  np.sqrt(d_x**2 + d_y**2) 

        # if self.next_goal is not None:
        #     i_opt_ind_for_loss = np.argmin(loss, axis=1) # list of indexes 
        
        loss = loss.min(axis=1)
        return loss



    def update_velocities(self, dt):
        vx = (self.curr_state.x - self.prev_state.x) / dt
        vy = (self.curr_state.y - self.prev_state.y) / dt
        v = math.sqrt(vx ** 2 + vy ** 2)
        alpha = math.atan2(vy, vx)
        v = v * math.cos(alpha - self.curr_state.yaw)

        # calculate robot ang velocity
        d_yaw = (self.curr_state.yaw - self.prev_state.yaw)
        d_yaw = (d_yaw + math.pi) % (2 * math.pi) - math.pi
        w = d_yaw / dt

        # update robot velocities
        self.robot.v = v
        self.robot.w = w

    def update_state(self, msg):
        """
        Update current robot state and velocities (v and w)
        """
        for item in msg.transforms:
            if self.last_tf_callback_time is None:
                self.last_tf_callback_time = time.time()
                break
            if (item.header.frame_id == self.parent_frame
                    and item.child_frame_id == self.robot_frame):
                # time delta calculation
                dt = time.time() - self.last_tf_callback_time
                # update previous state
                self.prev_state = self.curr_state
                # update robot state
                x, y = item.transform.translation.x, item.transform.translation.y
                yaw = quaternion_to_euler(item.transform.rotation)[0]
                self.curr_state = RobotState(x, y, yaw)
                self.robot.set_state(self.curr_state)
                # update velocity
                self.update_velocities(dt)
                # update last tf callback time
                self.last_tf_callback_time = time.time()
                # check if goal reached
                if self.got_path and self.robot.goal_reached(self.curr_goal):
                    if len(self.goal_queue) > 0:
                        # next goal
                        if self.next_goal is None:
                            self.next_goal = self.goal_queue.pop(0)
                        self.curr_goal = self.next_goal
                        self.next_goal = self.goal_queue.pop(0)
                        rospy.loginfo("New goal = " + self.curr_goal.to_str())
                    elif self.next_goal is not None:
                        self.curr_goal = self.next_goal
                        self.next_goal = None
                    else:
                        # end of trajectory
                        self.stop = True
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

    def publish_control_seq(self, controls):
        r = rospy.Rate(self.cmd_freq) 
        for control in controls:
            self.publish_control(control)
            r.sleep()


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
                i = i + 1
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


    def load_nn_model(self, model_path):
        """
        load NN model (.onnx) from given path, using nnio module
        """
        return nnio.ONNXModel(model_path)


def main():
    # model_path = sys.argv[1]
    mppic = MPPIController('mppic')
    mppic.wait_for_path()
    mppic.start()
    try:
        rospy.spin()
    except:
        pass


if __name__ == '__main__':
    main()
