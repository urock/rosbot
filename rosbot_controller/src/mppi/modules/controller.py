from utils.dtypes import Control
import rospy
from geometry_msgs.msg import Twist

import sys
sys.path.append("..")


class Controller:
    def __init__(self):
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.publish_time = 0.0

    def publish_control(self, control):
        """Publishes controls for the rosbot

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
        self.publish_time = rospy.Time.now()
    def publish_stop_control(self):
        self.publish_control(Control())
