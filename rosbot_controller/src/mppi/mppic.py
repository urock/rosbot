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

from mpc_dtypes import State, Control, dist_L2
from mpc_utils import quaternion_to_euler

from profiler import profile

class MPPIController:
    def __init__(self, node_name):
        rospy.init_node(node_name, anonymous=True)

        # get parameters
        self.model_path = rospy.get_param('~model_path', None)
        self.cmd_topic = rospy.get_param('~cmd_topic', "/cmd_vel")
        self.map_frame = rospy.get_param('~map_frame', "odom")
        self.base_frame = rospy.get_param('~base_frame', "base_link")
        self.cmd_freq = int(rospy.get_param('~cmd_freq', 30))  # Hz

        self.curr_state = State()
        self.prev_state = State()

        self.reference_traj = []
        self.traj_lookahead = 20
        self.curr_goal_idx = - 1 
        self.goal_tolerance = 0.2

        self.dt = 1.0 / self.cmd_freq
        self.rate = rospy.Rate(self.cmd_freq)

        # TODO make entity for this
        self.limit_v = 0.5
        self.timesteps_num = 50
        self.batch_size = 100  
        self.iter_count = 3
        self.v_std = 0.1  # standart deviation
        self.w_std = 0.1  # standart deviation
        self.goals_interval = 0.1
        self.model = nnio.ONNXModel(self.model_path)

        self.curr_control = np.zeros(shape = (self.batch_size, self.timesteps_num, 2))  

        self.got_path = False
        self.path_sub = rospy.Subscriber("/path", Path, self.path_cb)

        self.time_from_prev_tf_cb = time.time()
        self.tf_sub = rospy.Subscriber("/tf", TFMessage, self.tf_cb)

        rospy.Timer(rospy.Duration(self.dt), self.update_goal_cb)

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
        self.marker_pub = rospy.Publisher('/current_goal', Marker, queue_size=10)
        self.trajectories_pub = rospy.Publisher('/trajectories', MarkerArray, queue_size=10)
        self.curr_path_pub = rospy.Publisher("/current_path", Path, queue_size=5)


    def start(self):
        try:
            while not rospy.is_shutdown():
                if self.got_path:
                    self.run()
                else:
                    self.stop()
                self.rate.sleep()

        except KeyboardInterrupt:
            rospy.loginfo("Interrupted")


    def run(self):
        start = time.perf_counter()
        control = self.get_best_control()
        t =  time.perf_counter() - start

        steps_passed = round( (t / self.dt) )
        steps_passed = min(steps_passed, self.timesteps_num-1)
        v_best = control[0 + steps_passed, 0]
        w_best = control[0 + steps_passed, 1]

        rospy.loginfo_throttle(2, "[v,w] = [{:.2f} {:.2f}]. Exec Time {}, MPPI Step {} \n".format(v_best, w_best, t, steps_passed))
        self.publish_control(Control(v_best, w_best))

    def get_best_control(self):
        best_control = self.curr_control

        for _ in range(self.iter_count):
            control_seqs = best_control + self.generate_noise()
            control_seqs = np.clip(control_seqs, -self.limit_v, self.limit_v) # Clip both v and w ?

            init_states = self.create_init_state(control_seqs)
            predicted_velocities = self.predict_velocities(init_states)
            trajectories = self.predict_trajectories(predicted_velocities)
            curr_losses = self.calc_losses(predicted_velocities, trajectories)
            # self.visualize_trajs(trajectories)

            best_idx = np.argmin(curr_losses, axis=0)
            best_loss = curr_losses[best_idx]
            best_control = control_seqs[best_idx]
            if best_loss <= 0.05:
                break

        self.curr_control = best_control
        return best_control


    def generate_noise(self):
        v_noise = np.random.normal(0.0, self.v_std, size=(self.batch_size, self.timesteps_num, 1))
        w_noise = np.random.normal(0.0, self.w_std, size=(self.batch_size, self.timesteps_num, 1))
        return np.concatenate([v_noise, w_noise], axis=2)


    def create_init_state(self, control_seqs):
        shape = control_seqs.shape
        init_states = np.zeros( (shape[0], shape[1], 5) )

        init_states[:, 0, 0] = self.curr_state.v
        init_states[:, 0, 1] = self.curr_state.w
        init_states[:, :, 2:4] = control_seqs
        init_states[:, :, 4] = self.dt

        return init_states

    def predict_velocities(self, init_states):
        """ Filling initial states with predicted velocities along time horizont for all batches and returns predicted velocities

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
        v, w = velocities[:, :, 0], velocities[:, :, 1]
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

    # @profile("Loss function", 1)
    def calc_losses(self, velocities, trajectories):
        loss = np.zeros(shape = (trajectories.shape[0], trajectories.shape[1]))
        x = trajectories[:, :, 0]
        y = trajectories[:, :, 1]

        traj_end = len(self.reference_traj)
        end = self.curr_goal_idx + self.traj_lookahead + 1
        for q in range(self.curr_goal_idx, end):
            if q >= traj_end:
                break
            goal = self.reference_traj[q]
            loss += (x - goal.x)**2 + (y-goal.y)**2

        return loss.min(axis=1)

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
            self.curr_goal_idx = nearest_pt_idx
            self.publish_goal_marker(self.get_curr_goal(), 1000)
            return

        self.curr_goal_idx = nearest_pt_idx + 1
        if self.curr_goal_idx == len(self.reference_traj):
            self.curr_goal_idx = -1
            self.got_path = False
            return

        self.publish_goal_marker(self.get_curr_goal(), 1000)

    def is_goal_reached(self, dist):
        return (dist < self.goal_tolerance) and self.got_path 

    def get_nearest_traj_point_idx_and_dist(self, curr_state):
        min_idx = self.curr_goal_idx
        min_dist = 10e18

        traj_end = len(self.reference_traj)
        end = self.curr_goal_idx + self.traj_lookahead + 1
        for q in range(self.curr_goal_idx, end):
            if q >= traj_end:
                break

            curr_dist = dist_L2(curr_state, self.reference_traj[q])
            if min_dist >= curr_dist:
                min_dist = curr_dist
                min_idx = q

        return min_idx, min_dist

    def path_cb(self, msg):
        for pose in msg.poses:
            x, y = pose.pose.position.x, pose.pose.position.y
            yaw = quaternion_to_euler(pose.pose.orientation)[0]

            self.reference_traj.append(State(x, y, yaw))
        self.got_path = True
        self.curr_goal_idx = 0



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

    def stop(self):
        self.publish_control(Control())

    def visualize_trajs(self, trajectories):
        """ Publishes trajectories as arrays of marker points for visualization in Rviz

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
        self.trajectories_pub.publish(marker_array)

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

        self.curr_path_pub.publish(path)

    
    def publish_goal_marker(self, point, id):
        marker = Marker()
        marker.id = id
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "odom"
        marker.lifetime = rospy.Duration(0)
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale = Vector3(0.1, 0.1, 0.1)
        marker.color.r, marker.color.g, marker.color.a = (1.0, 1.0, 1.0)

        marker.pose.position.x = point.x
        marker.pose.position.y = point.y
        marker.pose.position.z = 0.05
        marker.pose.orientation.w = 0

        self.marker_pub.publish(marker)

def main():
    mppic = MPPIController('mppic')
    mppic.start()
    rospy.spin()

if __name__ == '__main__':
    main()


# Nearest loss. Too long execution
# loss = np.zeros(shape = (trajectories.shape[0], trajectories.shape[1]))
# for batch_idx in range(trajectories.shape[0]):
#     for time_step_idx in range(trajectories.shape[1]):
#         pt = trajectories[batch_idx, time_step_idx]
#         nearest_pt = self.get_nearest_traj_point(pt)
#         loss[batch_idx, time_step_idx] += self.np_dist_L2(pt, nearest_pt) 

# loss = np.apply_along_axis(trajectories, 1, self.get_trajectory_loss)

# loss = (2.5 * dx**2) + (2.5 * dy**2)  #+ (50 * w**2) + (v**2)

# Get N points 
# loss = np.zeros(shape = (trajectories.shape[0], trajectories.shape[1]))
# traj_end = len(self.reference_traj)
# end = self.curr_goal_idx + self.traj_lookahead + 1
# for q in range(self.curr_goal_idx, end):
#     if q >= traj_end:
#         break
#     goal = self.reference_traj[q]
#     loss +=  (x - goal.x)**2 + (y-goal.y)**2


# def get_trajectory_loss(self, trajectory):
#     loss = 0
#     for pt in trajectory:
#         nearest_pt = self.get_nearest_traj_point(pt)
#         loss += self.np_dist_L2(pt, nearest_pt) 
#     return loss

# def get_nearest_traj_point_idx(self, curr_point):
#     min_idx = self.curr_goal_idx
#     min_dist = 10e18

#     traj_end = len(self.reference_traj)
#     end = self.curr_goal_idx + self.traj_lookahead + 1
#     for q in range(self.curr_goal_idx, end):
#         if q >= traj_end:
#             break

#         curr_dist = self.np_dist_L2(curr_point, self.reference_traj[q])
#         if min_dist >= curr_dist:
#             min_dist = curr_dist
#             min_idx = q
#     return min_idx

# def np_dist_L2(self, np_lhs, rhs):
#     return (np_lhs[0] - rhs.x)** 2 + (np_lhs[1] - rhs.y)**2

# def get_nearest_traj_point(self, curr_point):
#     return self.reference_traj[self.get_nearest_traj_point_idx(curr_point)]


