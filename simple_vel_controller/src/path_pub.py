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


def main():
    rospy.init_node("path_pub", anonymous=True)
    rospy.loginfo("path_pub init")

    path_topic_name = "/path"

    path_pub = rospy.Publisher(path_topic_name, Path, queue_size=5)    

    msg = Path()
    msg.header.frame_id = "odom"
    msg.header.stamp = rospy.Time.now()
    cnt = 0
    msg.header.seq = cnt
    cnt += 1
    msg.poses = []

    x_ar = np.arange(0,2*np.pi,0.1)   # start,stop,step
    y_ar = np.sin(x_ar)

    # path = [(x_ar[i], y_ar[i]) for ]
    # path = [(2.0, 0), (2.0, 2.0), (0, 2.0), (0, 0)]

    # plt.plot(x,y)
    # plt.show()

    for i in range(len(x_ar)):
        ps = PoseStamped()
        ps.header = msg.header
        ps.header.seq = cnt
        cnt += 1
        ps.pose.position.x = x_ar[i]
        ps.pose.position.y = y_ar[i]
        ps.pose.position.z = 0 
        msg.poses.append(ps)        

    while not rospy.is_shutdown():
        if path_pub.get_num_connections() > 1:
            break
        rospy.sleep(0.3)

    path_pub.publish(msg)

if __name__ == '__main__':
    main()
