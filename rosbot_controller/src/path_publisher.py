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
from geometry_msgs.msg import PoseStamped, Quaternion


def IsValidTrajType(traj_type):
    return 'sin' in traj_type or traj_type in ('-line', 'from_file', 'polygon') or 'spiral' in traj_type

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


def SinTrajGenerator(msg, step, a=1.0, f=1.0, reverse=False):
    K = -1 if reverse==True else 1
    x_ar = np.arange(0, 2*np.pi * K, step * K, dtype=float)   # start,stop,step
    y_ar = float(a) * np.sin(float(f) * x_ar)
    yaw_arr = float(a) * float(f) * np.cos(float(f) * x_ar)

    cnt = 0
    for i in range(len(x_ar)):
        ps = PoseStamped()
        ps.header = msg.header
        ps.header.seq = cnt
        cnt += 1
        ps.pose.position.x = x_ar[i]
        ps.pose.position.y = y_ar[i]
        ps.pose.position.z = 0 
        ps.pose.orientation = euler_to_quaternion(yaw=math.atan(yaw_arr[i]), roll=0, pitch=0)
        msg.poses.append(ps)  

    return msg 

def PolygonTrajGenerator(msg, step):

    p_edges = [(2.0, -0.1), (2.1, 1.9),  (0.1, 2.0), (0, 0)] # square
     
    
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
        ps.pose.orientation = euler_to_quaternion(yaw=math.atan2(p[1], p[0]), roll=0, pitch=0)
        msg.poses.append(ps)  

    return msg     

def SpiralTrajGenerator(msg, step, amplitude):

    key_points = []
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
        y = r * math.sin(f)
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
    traj_type = traj_type.strip().split('sin') # []
    period = float(traj_type[-1])
    amplitude = float(traj_type[0].split("_")[-1])
    reverse = traj_type[0].split("_")[0] == 'reverse'
    return amplitude, period, reverse

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
        amplitude, freq, reverse = parse_sin_traj(traj_type)
        msg = SinTrajGenerator(msg, step, amplitude, freq, reverse)
    elif traj_type == 'polygon':
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


def euler_to_quaternion(yaw, pitch, roll):
    """
    Args:
        yaw: yaw angle
        pitch: pitch angle
        roll: roll angle
    Return:
        quaternion [qx, qy, qz, qw]
    """
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(
        pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(
        pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(
        pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(
        pitch / 2) * np.sin(yaw / 2)

    return Quaternion(x=qx, y=qy, z=qz, w=qw)


if __name__ == '__main__':
    main()
