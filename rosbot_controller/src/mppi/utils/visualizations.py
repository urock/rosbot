import rospy

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Vector3


def visualize_trajs(id_start, publisher, trajectories, point_coeff=0.3):
    marker_array = MarkerArray()
    i = id_start
    for traj in trajectories:
        step = int(len(traj)*point_coeff)
        for p in traj[::step]:
            marker = create_marker(i, p)
            marker_array.markers.append(marker)
            i = i + 1

    publisher.publish(marker_array)


def visualize_reference(id_start, publisher, reference_traj, goal_idx, traj_lookahead):
    marker_array = MarkerArray()
    i = id_start

    traj_end = reference_traj.shape[0]
    end = goal_idx + traj_lookahead + 1
    end = min(end, traj_end)

    for q in range(goal_idx, end):
        marker = create_marker(i, reference_traj[q])
        marker_array.markers.append(marker)
        i = i + 1

    publisher.publish(marker_array)


def create_marker(id, pt):
    marker = Marker()
    marker.id = id
    marker.header.stamp = rospy.Time.now()
    marker.header.frame_id = "odom"
    marker.lifetime = rospy.Duration(0)

    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.scale = Vector3(0.035, 0.035, 0.035)
    marker.color.r, marker.color.g, marker.color.a = (0.0, 1.0, 1.0)
    marker.pose.position.x = pt[0]
    marker.pose.position.y = pt[1]
    marker.pose.position.z = 0.05
    marker.pose.orientation.w = 0
    return marker
