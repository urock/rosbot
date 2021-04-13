import time
import math
import copy

import rospy
from tf2_msgs.msg import TFMessage

from utils.mpc_dtypes import State
from utils.mpc_utils import quaternion_to_euler


class Robot:
    """
    Class keeps state of the robot, updating it from external source (tf messages)
    """

    def __init__(self):
        self.tf_cb_time = time.time()
        self.tf_sub = rospy.Subscriber("/tf", TFMessage, self.tf_cb)
        self.map_frame = rospy.get_param('~map_frame', "odom")
        self.base_frame = rospy.get_param('~base_frame', "base_link")

        self.curr_state = State()
        self.prev_state = State()

    def tf_cb(self, msg):
        odom = self.get_odom_tf(msg)
        if not odom:
            return

        dt = self.get_diff_time()
        self.update_state(odom, dt)

    def get_odom_tf(self, msg):
        odom = None
        for transform in msg.transforms:
            if (transform.header.frame_id == self.map_frame
                    and transform.child_frame_id == self.base_frame):
                odom = transform
                break
        return odom

    def get_diff_time(self):
        cb_come_time = time.time()
        dt = cb_come_time - self.tf_cb_time
        self.tf_cb_time = cb_come_time
        return dt

    def update_state(self, odom, dt):
        self.curr_state.x = odom.transform.translation.x
        self.curr_state.y = odom.transform.translation.y
        self.curr_state.yaw = quaternion_to_euler(odom.transform.rotation)[0]
        self.update_velocities(dt)
        self.prev_state = copy.copy(self.curr_state)

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
