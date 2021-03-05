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
    return 'sin' in traj_type or traj_type in ('-line', 'from_file', 'complex') or 'spiral' in traj_type

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


def SinTrajGenerator(msg, step, a=1.0, f=1.0):
    x_ar = np.arange(0,2*np.pi, step, dtype=float)   # start,stop,step
    y_ar = float(a) * np.sin(float(f) * x_ar)

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

    # p_edges = [(2.0, -0.1), (2.1, 1.9),  (0.1, 2.0), (0, 0)] # square
    # p_edges = [(0.1, 2.1), (1.2, 0.0),  (1.3, 2.1), (2.5, 0.0), (2.7, 2.2), (3.7, 0.0), (3.9, 2.1), (4.1, 0.0)] # saw 
    # p_edges = [(0.1, -2.1), (-1.2, 0.0),  (-1.3, -2.1), (-2.5, 0.0), (-2.7, -2.2), (-3.7, 0.0), (-3.9, -2.1), (-4.1, 0.0)] # saw 
    # p_edges = [(-2.1, 0.1), (-2.2, 1.2),  (-2.6, 0.0), (-2.8, 1.8), (-2.9, 0.2), (-3.8, 2.5), (-3.9, 0.0), (-5.5, 3.0), (-6.0, 0.0)] # saw
    p_edges = [(0.001, 0.005), (-0.001, -0.005), (0.001, 0.005), (-0.001, -0.005)] # saw 
      
    
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

def SpiralTrajGenerator(msg, step, amplitude):

    key_points = []
    print(key_points)
    print(amplitude)
    if amplitude > 0:
        k = 1
    else:
        k = -1
    amplitude = abs(amplitude)
    f = 0
    while 1:
        # for (f in range (0 2*pi*N))
        r = step * math.exp(f*0.1)
        x = k * r * math.cos(f)
        y = k * r * math.sin(f)
        # print(x, y)
        if abs(x) > amplitude or abs(y) > amplitude:
            points = edges_to_points(key_points)
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
        else:
            key_points.append([x, y])
            f += 0.1

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
    a = coef[0] if coef[0] != '' or coef[0] is None else 1
    f = coef[1] if coef[1] != '' or coef[1] is None else 1
    return a, f

def parse_spiral_traj(traj_type):
    coef = traj_type.split('spiral')
    amp = coef[0] if coef[0] != '' or coef[0] is None else 1
    return float(amp)

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
    elif traj_type == '-line':
        msg = PolygonTrajGenerator(msg, step)
    elif traj_type == 'from_file':
        msg = FromFileTrajGenerator(msg, move_plan)
    elif 'spiral' in traj_type:
        amplitude = parse_spiral_traj(traj_type)
        msg = SpiralTrajGenerator(msg, step, amplitude)

    while not rospy.is_shutdown():
        if path_pub.get_num_connections() > 1:
            break
        rospy.sleep(0.3)

    rospy.sleep(2.0) # sometimes plotter_node can't get this message, and so we need some delay
    path_pub.publish(msg)
    return

if __name__ == '__main__':
    main()
