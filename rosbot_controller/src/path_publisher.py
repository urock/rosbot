#!/usr/bin/env python
# license removed for brevity
import rospy
import os
import roslib
import time
import math
import numpy as np
import pathlib
from nav_msgs.msg import Path
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped

def IsValidTrajType(traj_type):
    return 'sin' in traj_type or traj_type in ('polygon', 'from_file', 'complex')

def parse_plan(file_name):
    edges = list()

    with (pathlib.Path(file_name)).open() as f:
        for line in f:
            p = line.replace('\n', '')
            if p:
                p = p.split(" ")
                edges.append((float(p[0]), float(p[1])))

    return edges

def edges_to_points(edges):
    points = list()
    p1 = (0.0, 0.0)

    for edge in edges:
        p2 = edge
        k = (p2[1] - p1[1]) / (p2[0] - p1[0])
        b = (p1[1]*p2[0] - p2[1]*p1[0] ) / (p2[0] - p1[0])
        x = p1[0]
        y = k*x + b
        points.append((x,y))
        step = abs(p2[0] - p1[0])/10
        if p2[0] > p1[0]:
            while (x < p2[0]):
                x += step
                if (x > p2[0]):
                    break
                y = k*x + b
                points.append((x,y))
        else:
             while (x > p2[0]):
                x -= step
                if (x < p2[0]):
                    break
                y = k*x + b
                points.append((x,y))
        p1 = p2

    return points


def SinTrajGenerator(msg, step, a=1, f=1):
    x_ar = np.arange(0,2*np.pi, step)   # start,stop,step
    y_ar = int(a) * np.sin(int(f) * x_ar)

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

def PolygonTrajGenerator(msg, step):

    p_edges = [(2.0, -0.1), (2.1, 1.9),  (0.1, 2.0), (0, 0)]
    
    points = edges_to_points(p_edges)
    
    cnt = 0
    for p in points:
        ps = PoseStamped()
        ps.header = msg.header
        ps.header.seq = cnt
        cnt += 1
        ps.pose.position.x = p[0]
        ps.pose.position.y = p[1]
        ps.pose.position.z = 0 
        msg.poses.append(ps)  

    return msg     


def FromFileTrajGenerator(msg, move_plan):
    if move_plan is None:
        rospy.logerr("Move plan was not specified")
        return 1
    
    edges = parse_plan(move_plan)

    points = edges_to_points(edges)

    cnt = 0
    for p in points:
        ps = PoseStamped()
        ps.header = msg.header
        ps.header.seq = cnt
        cnt += 1
        ps.pose.position.x = p[0]
        ps.pose.position.y = p[1]
        ps.pose.position.z = 0 
        msg.poses.append(ps)   
    
    return msg


def parse_sin_traj(traj_type):
    coef = traj_type.split('sin')
    a = coef[0] if coef[0] != '' else 1
    f = coef[1] if coef[1] != '' else 1
    return a, f

def main():
    rospy.init_node("path_pub", anonymous=True)
    rospy.loginfo("path_pub init")
    traj_type = rospy.get_param('~traj_type')
    move_plan = rospy.get_param('~move_plan')

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

    if 'sin' in traj_type:
        print("TRY PARSE", traj_type )
        amplitude, freq = parse_sin_traj(traj_type)
        msg = SinTrajGenerator(msg, step, amplitude, freq)
    elif traj_type == 'polygon':
        msg = PolygonTrajGenerator(msg, step)
    elif traj_type == 'from_file':
        msg = FromFileTrajGenerator(msg, move_plan)

    while not rospy.is_shutdown():
        if path_pub.get_num_connections() > 1:
            break
        rospy.sleep(0.3)

    rospy.sleep(2.0) # sometimes plotter_node can't get this message, and so we need some delay
    path_pub.publish(msg)
    return

if __name__ == '__main__':
    main()
