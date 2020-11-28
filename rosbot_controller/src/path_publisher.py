#!/usr/bin/env python
# license removed for brevity
import rospy
import os
import roslib
import time
import math
import numpy as np

from nav_msgs.msg import Path
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped

def IsValidTrajType(traj_type):
    return traj_type in ('sin', 'polygon')

def SinTrajGenerator(msg, step):
    x_ar = np.arange(0,2*np.pi, step)   # start,stop,step
    y_ar = np.sin(x_ar)

    cnt = 0
    for i in range(len(x_ar)):
        ps = PoseStamped()
        ps.header = msg.header
        ps.header.seq = cnt
        cnt += 1
        ps.pose.position.x = x_ar[i]
        ps.pose.position.y = y_ar[i]
        ps.pose.position.z = 0 
        msg.poses.append(ps)  

    return msg  


def main():
    rospy.init_node("path_pub", anonymous=True)
    rospy.loginfo("path_pub init")
    traj_type = rospy.get_param('~traj_type')

    if not IsValidTrajType(traj_type):
        rospy.logerr("Not valid traj type")
        return

    path_topic_name = "/path"

    path_pub = rospy.Publisher(path_topic_name, Path, queue_size=5)    

    msg = Path()
    msg.header.frame_id = "odom"
    msg.header.stamp = rospy.Time.now()
    msg.header.seq = 0
    msg.poses = []

    step = 0.1

    if traj_type == 'sin':
        msg = SinTrajGenerator(msg, step)    

    while not rospy.is_shutdown():
        if path_pub.get_num_connections() > 1:
            break
        rospy.sleep(0.3)

    path_pub.publish(msg)

if __name__ == '__main__':
    main()
