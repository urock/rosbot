#!/usr/bin/env python3

import time
import math

import numpy as np
import nnio

import rospy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped, Twist, Pose, Point
from geometry_msgs.msg import Vector3, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray

from mpc_dtypes import State, Control, dist_L2, dist_L2_np
from mpc_utils import quaternion_to_euler
from models.rosbot import Rosbot

from losses import sum_loss, order_loss, nearest_loss


class MPPIController:
    def __init__(self, model, loss):
        self.cmd_topic = rospy.get_param('~cmd_topic', "/cmd_vel")
        self.map_frame = rospy.get_param('~map_frame', "odom")
        self.base_frame = rospy.get_param('~base_frame', "base_link")
        self.cmd_freq = int(rospy.get_param('~cmd_freq', 30))  # Hz

        self.curr_state = State()
        self.prev_state = State()

        self.reference_traj = np.empty(shape=(0, 3))
        self.traj_lookahead = 15
        self.curr_goal_idx = - 1
        self.goal_tolerance = 0.2
        self.goals_interval = 0.1

        self.dt = 1.0 / self.cmd_freq
        self.rate = rospy.Rate(self.cmd_freq)

        self.limit_v = 0.5
        self.preferable_speed = 0.5
        self.time_steps = int(int(self.traj_lookahead * self.goals_interval /
                                  self.preferable_speed) / self.dt)

        self.batch_size = 100
        self.iter_count = 1
        self.v_std = 0.1  # standart deviation
        self.w_std = 0.3  # standart deviation
        self.model = model
        self.loss = loss

        self.control_matrix = np.zeros(shape=(self.batch_size, self.time_steps, 5))
        self.control_matrix[:, :, 4] = self.dt
        self.velocities = self.control_matrix[:, :, :2].view()  # v,w
        self.controls = self.control_matrix[:, :, 2:4].view()  # v, w

        self.curr_control = np.zeros(shape=(self.time_steps, 2))

        self.got_path = False
        self.path_sub = rospy.Subscriber("/path", Path, self.path_cb)

        self.time_from_prev_tf_cb = time.time()
        self.tf_sub = rospy.Subscriber("/tf", TFMessage, self.tf_cb)

        rospy.Timer(rospy.Duration(self.dt), self.update_goal_cb)

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
        self.trajs_pub = rospy.Publisher('/mppi_trajs', MarkerArray, queue_size=10)
        self.ref_pub = rospy.Publisher('/ref_trajs', MarkerArray, queue_size=10)
        self.path_pub = rospy.Publisher("/mppi_path", Path, queue_size=5)

    def start(self):
        """Starts main loop running mppi controller if got path.
        """
        while not rospy.is_shutdown():
            goal_idx = self.curr_goal_idx
            if self.got_path and goal_idx != -1:
                control = self.get_best_control(goal_idx)
                self.rate.sleep()
                self.publish_control(control)
            else:
                self.stop()
                self.rate.sleep()

    def stop(self):
        self.publish_control(Control())

    def publish_control(self, control):
        """ Publishes controls for the rosbot

        Args:
            control: control vector of Control type
        """
        cmd = Twist()
        cmd.linear.x = control.v
        cmd.linear.y = 0
        cmd.linear.z = 0
        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = control.w
        self.cmd_pub.publish(cmd)

    def get_best_control(self, goal_idx):
        """Calculates next best control using mppic algorithm

        Return: Control - v, w
        """

        best_control = None
        iter = -1

        start = time.perf_counter()
        t = start
        for iter in range(self.iter_count):
            control_seqs = self.generate_next_control_seqs()
            self.update_init_state(control_seqs)
            self.predict_velocities()
            trajectories = self.predict_trajectories()
            losses = self.loss(trajectories, self.reference_traj, self.traj_lookahead, goal_idx)
            best_idx = np.argmin(losses, axis=0)
            best_loss = losses[best_idx]
            best_control = control_seqs[best_idx]

            self.visualize_trajs(trajectories)
            t = time.perf_counter() - start
            if (best_loss <= 0.05):
                break

        self.curr_control = best_control
        v_best = best_control[0 + round(t / self.dt), 0]
        w_best = best_control[0 + round(t / self.dt), 1]

        rospy.loginfo_throttle(
            2, "Iter: {}. Exec Time {}.  [v, w] = [{:.2f} {:.2f}].  \n".format(iter + 1, t, v_best, w_best))
        return Control(v_best, w_best)

    def generate_next_control_seqs(self):
        """ 
        Return:
            Randomly generates control sequences with means in curr_control - 
            np.array of shape [batch_size, time_steps, 2] where 2 is for v, w respectively
        """
        v_noises = np.random.normal(0.0, self.v_std, size=(self.batch_size, self.time_steps, 1))
        w_noises = np.random.normal(0.0, self.w_std, size=(self.batch_size, self.time_steps, 1))
        noises = np.concatenate([v_noises, w_noises], axis=2)
        next_seqs = self.curr_control[np.newaxis] + noises

        next_seqs = np.clip(next_seqs, -self.limit_v, self.limit_v)  # Clip both v and w ?
        return next_seqs

    def update_init_state(self, control_seqs):
        """ Updates current control_matrix velocities and controls

        Args:
            control_seqs: np.array of shape [batch_size, time_steps, 2] where 2 is for v, w 
        """
        self.velocities[:, 0, 0] = self.curr_state.v
        self.velocities[:, 0, 1] = self.curr_state.w
        self.controls[:, :] = control_seqs

    def predict_velocities(self):
        """ Fills in control_matrix with predicted velocities

        Args:
            [in] init_states: np.array of shape [batch, time_steps, state_dim + control_dim + 1] where 1 is for dt 
        """
        time_steps = self.control_matrix.shape[1]
        for t_step in range(time_steps - 1):
            curr_batch = self.control_matrix[:, t_step].astype(np.float32)
            curr_predicted = self.model(curr_batch)
            self.velocities[:, t_step + 1] = curr_predicted

    def predict_trajectories(self):
        """ Propagetes trajectories using control matrix velocities and current state

        Return:
            trajectory points - np.array of shape [batch_size, time_steps, 3] where 3 is for x, y, yaw respectively
        """
        v, w = self.velocities[:, :, 0], self.velocities[:, :, 1]
        current_yaw = self.curr_state.yaw
        yaw = np.cumsum(w * self.dt, axis=1)
        yaw += current_yaw - yaw[:, :1]
        v_x = v * np.cos(yaw)
        v_y = v * np.sin(yaw)
        x = np.cumsum(v_x * self.dt, axis=1)
        y = np.cumsum(v_y * self.dt, axis=1)
        x += self.curr_state.x - x[:, :1]
        y += self.curr_state.y - y[:, :1]

        traj_points = np.concatenate([
            x[:, :, np.newaxis],
            y[:, :, np.newaxis],
            yaw[:, :, np.newaxis],
        ], axis=2)
        return traj_points

    def get_curr_goal(self):
        return self.reference_traj[self.curr_goal_idx]

    def tf_cb(self, msg):
        odom = self.get_odom_tf(msg)
        if not odom:
            return

        dt = self.get_tf_cb_diff_time()
        self.update_state(odom, dt)

    def get_odom_tf(self, msg):
        odom = None
        for transform in msg.transforms:
            if (transform.header.frame_id == self.map_frame
                    and transform.child_frame_id == self.base_frame):
                odom = transform
        return odom

    def get_tf_cb_diff_time(self):
        cb_come_time = time.time()
        dt = cb_come_time - self.time_from_prev_tf_cb
        self.time_from_prev_tf_cb = cb_come_time
        return dt

    def update_state(self, odom, dt):
        self.curr_state.x = odom.transform.translation.x
        self.curr_state.y = odom.transform.translation.y
        self.curr_state.yaw = quaternion_to_euler(odom.transform.rotation)[0]
        self.update_velocities(dt)
        self.prev_state = self.curr_state

    def update_velocities(self, dt):
        v_x = (self.curr_state.x - self.prev_state.x) / dt
        v_y = (self.curr_state.y - self.prev_state.y) / dt
        v_linear = math.sqrt(v_x ** 2 + v_y ** 2)
        alpha = math.atan2(v_y, v_x)
        v = v_linear * math.cos(alpha - self.curr_state.yaw)

        d_yaw = (self.curr_state.yaw - self.prev_state.yaw)
        d_yaw = (d_yaw + math.pi) % (2 * math.pi) - math.pi
        w = d_yaw / dt

        return v, w

    def update_goal_cb(self, timer):
        if self.curr_goal_idx == -1:
            return

        nearest_pt_idx, dist = self.get_nearest_traj_point_idx_and_dist(self.curr_state)
        if not self.is_goal_reached(dist):
            # self.curr_goal_idx = nearest_pt_idx
            self.visualize_reference()
            return

        self.curr_goal_idx = self.curr_goal_idx + 1
        if self.curr_goal_idx == self.reference_traj.shape[0]:
            self.curr_goal_idx = -1
            self.got_path = False
            return

        self.visualize_reference()

    def is_goal_reached(self, dist):
        return (dist < self.goal_tolerance) and self.got_path

    def get_nearest_traj_point_idx_and_dist(self, curr_state):
        min_idx = self.curr_goal_idx
        min_dist = 10e18

        traj_end = self.reference_traj.shape[0]
        end = self.curr_goal_idx + self.traj_lookahead + 1
        for q in range(self.curr_goal_idx, min(end, traj_end)):
            curr_dist = dist_L2_np(curr_state, self.reference_traj[q])
            if min_dist >= curr_dist:
                min_dist = curr_dist
                min_idx = q

        return min_idx, min_dist

    def path_cb(self, msg):
        for pose in msg.poses:
            x, y = pose.pose.position.x, pose.pose.position.y
            yaw = quaternion_to_euler(pose.pose.orientation)[0]
            self.reference_traj = np.append(self.reference_traj, [[x, y, yaw]], axis=0)

        self.got_path = True
        self.curr_goal_idx = 0

    def visualize_trajs(self, trajectories):
        """ Publishes trajectories as arrays of marker points for visualization in Rviz
        """
        marker_array = MarkerArray()
        i = 0
        for traj in trajectories:
            step = int(len(traj)*0.3)
            for p in traj[::step]:
                marker = Marker()
                marker.id = i
                marker.header.stamp = rospy.Time.now()
                marker.header.frame_id = "odom"
                marker.lifetime = rospy.Duration(0)
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.scale = Vector3(0.05, 0.05, 0.05)
                marker.color.r, marker.color.g, marker.color.a = (0.0, 1.0, 1.0)
                marker.pose.position.x = p[0]
                marker.pose.position.y = p[1]
                marker.pose.position.z = 0.05
                marker.pose.orientation.w = 0
                i = i + 1
                marker_array.markers.append(marker)
        self.trajs_pub.publish(marker_array)

    def visualize_reference(self):
        """ Publishes trajectories as arrays of marker points for visualization in Rviz
        """
        marker_array = MarkerArray()
        i = 5000

        traj_end = self.reference_traj.shape[0]
        end = self.curr_goal_idx + self.traj_lookahead + 1
        for q in range(self.curr_goal_idx, end):
            if q >= traj_end:
                break
            marker = Marker()
            marker.id = i
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = "odom"
            marker.lifetime = rospy.Duration(0)
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.scale = Vector3(0.05, 0.05, 0.05)
            marker.color.r, marker.color.g, marker.color.a = (1.0, 0.0, 1.0)
            marker.pose.position.x = self.reference_traj[q][0]
            marker.pose.position.y = self.reference_traj[q][1]
            marker.pose.position.z = 0.05
            marker.pose.orientation.w = 0
            i = i + 1
            marker_array.markers.append(marker)
        self.ref_pub.publish(marker_array)

    def pubish_traj(self, traj):
        """
        Publishes a path as a message with
        the 'Path' data type for visualization in Rviz
        Args:
            traj (np.array shape = [timesteps_num, 2]): trajectory
        """
        path = Path()
        path.header.frame_id = self.map_frame
        tx, ty = traj[:, 0], traj[:, 1]

        for i in range(len(tx)):
            path.poses.append(PoseStamped(pose=Pose(position=Point(x=tx[i], y=ty[i], z=0))))

        self.path_pub.publish(path)


def main():
    rospy.init_node('mppic', anonymous=True)
    model_path = rospy.get_param('~model_path', None)
    model = nnio.ONNXModel(model_path)

    loss = nearest_loss
    mppic = MPPIController(model, loss)
    mppic.start()
    rospy.spin()


if __name__ == '__main__':
    main()
