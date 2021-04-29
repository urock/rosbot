import time
import math
from copy import copy

import rospy
from tf2_msgs.msg import TFMessage

from utils.dtypes import State
from utils.geometry import quaternion_to_euler


class Odom:
    """
    Class keeps state of the robot, updating it from external source (tf message)
    """

    def __init__(self):
        self.map_frame = rospy.get_param('~robot/map_frame', "odom")
        self.base_frame = rospy.get_param('~robot/base_frame', "base_link")

        self.curr_state = State()
        self.prev_state = State()

        self.tf_sub = rospy.Subscriber("/tf", TFMessage, self._tf_cb)
        self.tf_cb_time = time.time()

    def _tf_cb(self, msg):
        odom = self._get_odom_tf(msg)
        if not odom:
            return

        dt = self._get_diff_time()
        self._update_state(odom, dt)

    def _get_odom_tf(self, msg):
        odom = None
        for transform in msg.transforms:
            if (transform.header.frame_id == self.map_frame
                    and transform.child_frame_id == self.base_frame):
                odom = transform
                break
        return odom

    def _get_diff_time(self):
        come_time = time.time()
        dt = come_time - self.tf_cb_time
        self.tf_cb_time = come_time
        return dt

    def _update_state(self, odom, dt):
        self.curr_state.x = odom.transform.translation.x
        self.curr_state.y = odom.transform.translation.y
        self.curr_state.yaw = quaternion_to_euler(odom.transform.rotation)[0]
        self._update_velocities(dt)
        self.prev_state = copy(self.curr_state)

    def _update_velocities(self, dt):
        v_x = (self.curr_state.x - self.prev_state.x) / dt
        v_y = (self.curr_state.y - self.prev_state.y) / dt
        v_linear = math.sqrt(v_x ** 2 + v_y ** 2)
        alpha = math.atan2(v_y, v_x)
        v = v_linear * math.cos(alpha - self.curr_state.yaw)

        d_yaw = (self.curr_state.yaw - self.prev_state.yaw)
        d_yaw = (d_yaw + math.pi) % (2 * math.pi) - math.pi
        w = d_yaw / dt

        self.curr_state.v = v
        self.curr_state.w = w
