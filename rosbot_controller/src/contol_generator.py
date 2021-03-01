#!/usr/bin/env python
import rospy
import argparse
from geometry_msgs.msg import Twist
N = 0
V = -0.5
W = 0.5
FROM_FILE = False
cmd_list = list()

def timer_callback(event):
    global N, V, W, FROM_FILE, cmd_list
    if FROM_FILE:
        pass
    else:
        twist_cmd = Twist()
        #  forward moving
        if N >= 0:
            if N < 500:
                V = V + 0.001
                W = W + 0.002
            else:
                V = V - 0.001
                W = W - 0.002
            twist_cmd.linear.x = V
            twist_cmd.angular.z = W
            N = N + 1
        if N > 1000:
            N = -1
            V = -0.5
            W = 0.5

        #  backward moving
        if N < 0:
            if N > -500:
                V = V + 0.001
                W = W + 0.002
            else:
                V = V - 0.001
                W = W - 0.002
            twist_cmd.linear.x = -V
            twist_cmd.angular.z = -W
            N = N - 1
        if N < -1000:
            N = 0
            V = -0.5
            W = 0.5

        cmd_pub.publish(twist_cmd)


rospy.init_node("control_generator", anonymous=True)
cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)

# parse args
parser = argparse.ArgumentParser()
parser.add_argument('-file_path', action='store', dest='file_path',
                    required=False, help='Amplitude')
args = parser.parse_args()
FROM_FILE = True if args.file_path is not None else False

if FROM_FILE:
    import random
    with open(args.file_path) as f:
        f.readline()
        lines = f.readlines()
        for i in range(len(lines[0:-1])):
            # print(lines[i].split(" "))
            cur_t, v, z = lines[i].rstrip().split(" ")
            cur_t, v, z = float(cur_t), float(v), float(z)
            next_t = float(lines[i+1].split(" ")[0])
            dt = next_t - cur_t
            twist_cmd = Twist()
            twist_cmd.linear.x = v     # random() return value from 0 to 1
            twist_cmd.angular.z = z
            cmd_pub.publish(twist_cmd)
            ### remove it
            # dt = 0.03
            rospy.sleep(dt)
        twist_cmd.linear.x = 0
        twist_cmd.angular.z = 0
        cmd_pub.publish(twist_cmd)
else:
    rospy.Timer(rospy.Duration(0.033), timer_callback)

rospy.spin()