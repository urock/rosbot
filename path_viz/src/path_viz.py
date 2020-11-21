#!/usr/bin/env python
# license removed for brevity
import rospy
import roslib
import tf
import time
from tf2_msgs.msg import TFMessage

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path



class path_viz:

	def __init__(self, node_name):
		self.node_name = node_name
		rospy.init_node(self.node_name, anonymous=True)

		# fetch params from the ~private namespace
		self.origin_frame = rospy.get_param('~origin_frame')
		rospy.loginfo("%s is %s", rospy.resolve_name('~origin_frame'), self.origin_frame)
		self.base_link_frame = rospy.get_param('~base_link_frame')
		rospy.loginfo("%s is %s", rospy.resolve_name('~base_link_frame'), self.base_link_frame)

		self.tf_listener = tf.TransformListener()
		self.path_pub = rospy.Publisher('/base_link_path', Marker, queue_size=1)
		# self.goal_topic_viz = rospy.Publisher('/fast_sense/goal_viz', Marker, queue_size=1)
		self.path_len_ = 0

		self.current_pose = Pose()
		self.last_pose = Pose()
		self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

		# self.goal_topic_sub = rospy.Subscriber("/mover/path", Path, self.goal_callback)


	def publish_pose_marker(self):

		pMarker = Marker()
		pMarker.header.frame_id = self.origin_frame
		pMarker.header.stamp    = rospy.Time.now()
		pMarker.id = self.path_len_
		pMarker.type = Marker.SPHERE  # LINE_STRIP
		pMarker.action = Marker.ADD # ADD
		pMarker.pose.position = self.current_pose.position
		pMarker.pose.orientation.w = 1.0
		pMarker.scale.x = 0.03
		pMarker.scale.y = 0.03
		pMarker.scale.z = 0.03

		pMarker.color.r = 1.0
		pMarker.color.g = 0.0
		pMarker.color.b = 0.0
		pMarker.color.a = 1.0

		pMarker.points.append(self.last_pose.position)
		pMarker.points.append(self.current_pose.position)

		pMarker.lifetime = rospy.Duration(0)

		self.path_pub.publish(pMarker)

		self.path_len_ = self.path_len_ + 1
		self.last_pose = self.current_pose 


	def get_pose(self):
		pose = Pose()

		try:
			(coord,orient) = self.tf_listener.lookupTransform(self.origin_frame, self.base_link_frame , rospy.Time(0)) 
			pose.position.x = float(coord[0])
			pose.position.y = float(coord[1])
			pose.position.z = float(coord[2])
			pose.orientation.x = float(orient[0])
			pose.orientation.y = float(orient[1])
			pose.orientation.z = float(orient[2])
			pose.orientation.w = float(orient[3])
			self.current_pose = pose

			return True
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			return False

	def log_pose(self):
		ret_val = self.get_pose()
		if ret_val:
			self.publish_pose_marker()

	def timer_callback(self,timer):
		self.log_pose()

	def goal_callback(self, msg):
		for point in msg.poses:
			pMarker = Marker()
			pMarker.header.frame_id = self.origin_frame
			pMarker.header.stamp = rospy.Time.now()
			pMarker.id = 1
			pMarker.type = Marker.ARROW  # LINE_STRIP
			pMarker.action = Marker.ADD # ADD
			pMarker.pose.position = point.pose.position
			pMarker.pose.orientation = point.pose.orientation
			pMarker.scale.x = 0.3
			pMarker.scale.y = 0.05
			pMarker.scale.z = 0.07
	
			pMarker.color.r = 1.0
			pMarker.color.g = 1.0
			pMarker.color.b = 0.0
			pMarker.color.a = 1.0
			self.goal_topic_viz.publish(pMarker)


def main():
	viz = path_viz('path_viz')
	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("path_viz: Shutting down")


if __name__ == '__main__':
	main()