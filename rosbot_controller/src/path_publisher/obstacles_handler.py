#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import  OccupancyGrid
import rospy

class ObstacleHandler:
    def __init__(self, map_frame, obstacle_topic):
        self.map_frame = map_frame 
        self.obstacle_topic = obstacle_topic
        self.obstacles_pub = rospy.Publisher(self.obstacles_topic, OccupancyGrid, queue_size=5)
