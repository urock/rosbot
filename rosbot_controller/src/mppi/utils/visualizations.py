def visualize_trajs(self, trajectories):
    """ Publishes trajectories as arrays of marker points for visualization in Rviz
    """
    marker_array = MarkerArray()
    i = 0
    for traj in trajectories:
        step = int(len(traj)*0.3)
        for p in traj[::step]:
            marker = Marker()
            marker.id = i
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = "odom"
            marker.lifetime = rospy.Duration(0)
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.scale = Vector3(0.05, 0.05, 0.05)
            marker.color.r, marker.color.g, marker.color.a = (0.0, 1.0, 1.0)
            marker.pose.position.x = p[0]
            marker.pose.position.y = p[1]
            marker.pose.position.z = 0.05
            marker.pose.orientation.w = 0
            i = i + 1
            marker_array.markers.append(marker)
    self.trajs_pub.publish(marker_array)


def visualize_reference(self):
    """ Publishes trajectories as arrays of marker points for visualization in Rviz
    """
    marker_array = MarkerArray()
    i = 5000

    traj_end = self.reference_traj.shape[0]
    end = self.curr_goal_idx + self.traj_lookahead + 1
    for q in range(self.curr_goal_idx, end):
        if q >= traj_end:
            break
        marker = Marker()
        marker.id = i
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "odom"
        marker.lifetime = rospy.Duration(0)
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale = Vector3(0.05, 0.05, 0.05)
        marker.color.r, marker.color.g, marker.color.a = (1.0, 0.0, 1.0)
        marker.pose.position.x = self.reference_traj[q][0]
        marker.pose.position.y = self.reference_traj[q][1]
        marker.pose.position.z = 0.05
        marker.pose.orientation.w = 0
        i = i + 1
        marker_array.markers.append(marker)
    self.ref_pub.publish(marker_array)
