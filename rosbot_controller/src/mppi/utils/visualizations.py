import rospy

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Vector3


class Colors:
    red = [1, 0, 0, 1]
    green = [0, 1, 0, 1]
    blue = [0, 0, 1, 1]
    yellow = [1, 1, 0, 1]
    teal = [0, 1, 1, 1]
    purple = [1, 0, 1, 1]


class ReferenceVisualizer:
    def __init__(self, topic):
        self._curr_id = 0
        self._pub = rospy.Publisher(topic, MarkerArray, queue_size=10)

    def visualize(self, reference_trajectory, color, scale):
        self._curr_id = 0
        marker_array = MarkerArray()
        for q in range(len(reference_trajectory)):
            marker = create_marker(self._curr_id, reference_trajectory[q], color, scale)
            marker_array.markers.append(marker)
            self._curr_id += 1

        self._pub.publish(marker_array)

    def reset(self):
        marker_arr = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.action = Marker.DELETEALL
        marker_arr.markers.append(marker)

        self._pub.publish(marker_arr)
        self._curr_id = 0


class TrajectoriesVisualizer:
    def __init__(self, topic):
        self._curr_id = 0
        self._markers = MarkerArray()
        self._pub = rospy.Publisher(topic, MarkerArray, queue_size=100)

    def visualize(self):
        self._pub.publish(self._markers)
        self._markers = MarkerArray()
        self._curr_id = 0

    def add(self, trajectories, color, scale, step=1):
        for traj in trajectories[::step]:
            for point in traj:
                marker = create_marker(self._curr_id, point, color, scale)
                self._markers.markers.append(marker)
                self._curr_id += 1

    def reset(self):
        marker_arr = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.action = Marker.DELETEALL
        marker_arr.markers.append(marker)
        self._pub.publish(marker_arr)
        self._curr_id = 0


class StateVisualizer:
    def __init__(self, topic):
        self._curr_id = 0
        self._pub = rospy.Publisher(topic, Marker, queue_size=1)

    def visualize(self, state, color, scale):
        pt = [state.x, state.y]
        marker = create_marker(self._curr_id, pt, color, scale)
        self._pub.publish(marker)
        self._curr_id += 1

    def reset(self):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.action = Marker.DELETEALL
        self._pub.publish(marker)
        self._curr_id = 0


class ObstaclesVisualizer:
    def __init__(self, topic):
        self._curr_id = 0
        self._markers = MarkerArray()
        self._pub = rospy.Publisher(topic, MarkerArray, queue_size=100)

    def visualize(self, obstacles, color):
        for obstacle in obstacles:
            self.add(obstacle, color)

        self._pub.publish(self._markers)
        self._markers = MarkerArray()
        self._curr_id = 0

    def add(self, obstacle, color):
        pt = [obstacle[0], obstacle[1]]
        r = obstacle[2]

        marker = create_marker(self._curr_id, pt, color, scale=Vector3(r, r, 0.2))
        self._markers.markers.append(marker)
        self._curr_id += 1

    def reset(self):
        marker_arr = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.action = Marker.DELETEALL
        marker_arr.markers.append(marker)
        self._pub.publish(marker_arr)
        self._curr_id = 0


def create_marker(id, pt, color, scale=Vector3(0.035, 0.035, 0.035)):
    marker = Marker()
    marker.id = id
    marker.header.stamp = rospy.Time.now()
    marker.header.frame_id = "odom"
    marker.lifetime = rospy.Duration(0)

    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.scale = scale
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
    marker.pose.position.x = pt[0]
    marker.pose.position.y = pt[1]
    marker.pose.position.z = 0.05
    marker.pose.orientation.w = 0
    return marker
