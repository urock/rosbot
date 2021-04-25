import rospy
import numpy as np


class MetricHandler():
    def __init__(self, metric):
        self.metric = metric

    def show_metrics(self, time, path, reference_trajectory, offset):
        path_len = len(path) - offset
        lin_vels = path[:-offset, 3]
        ang_vels = path[:-offset, 4]

        value = self.metric(reference_trajectory, path)
        rospy.loginfo("Path Total Time: {:.6f}.".format(time))
        rospy.loginfo("Path Error by {}: {:.6f}.".format(self.metric.__name__, value))
        rospy.loginfo(
            "Mean velocities v = {:.6f}, w = {:.6f}.".format(np.mean(lin_vels), np.mean(ang_vels))
        )

    # path_rng = np.arange(path_len)
    # plt.figure(1)
    # plt.subplot(221)
    # plt.plot(path_rng, lin_vels)
    # plt.yscale("linear")
    # plt.title("Linear velocitie")
    # plt.xlabel("point")
    # plt.ylabel("Linear vel")

    # plt.subplot(222)
    # plt.plot(path_rng, ang_vels)
    # plt.yscale("linear")
    # plt.title("Angular")
    # plt.xlabel("point")
    # plt.ylabel("Angular vel")

    # control_len = len(self.controls)
    # control_rng = np.arange(control_len)
    # lin_controls = self.controls[:, 0]
    # ang_controls = self.controls[:, 1]

    # plt.subplot(223)
    # plt.plot(control_rng, lin_controls)
    # plt.yscale("linear")
    # plt.title("Linear Control")
    # plt.xlabel("point")
    # plt.ylabel("Linear vel")

    # plt.subplot(224)
    # plt.plot(control_rng, ang_controls)
    # plt.yscale("linear")
    # plt.title("Angular Control")
    # plt.xlabel("point")
    # plt.ylabel("Angular vel")
    # plt.show()
