import rospy
import numpy as np
import matplotlib.pyplot as plt


class MetricHandler():
    def __init__(self, metric):
        self.metric = metric
        self.path = np.zeros(shape=(0, 5))

    def add_state(self, state):
        self.path = np.append(self.path, state.to_numpy()[np.newaxis], axis=0)

    def show_metrics(self, time, controls, reference_trajectory):
        lin_vels = self.path[:, 3]
        ang_vels = self.path[:, 4]

        lin_controls = controls[:, 0]
        ang_controls = controls[:, 1]

        value = self.metric(reference_trajectory, self.path)
        rospy.loginfo("Path Total Time: {:.6f}.".format(time))
        rospy.loginfo("Path Error by {}: {:.6f}.\n".format(self.metric.__name__, value))

        self.show_statistics('Vels', lin_vels, ang_vels)
        self.show_statistics('Controls', lin_controls, ang_controls)

        self.show_graphs(lin_vels, ang_vels, controls)

    def show_statistics(self, tag, lin, ang):
        rospy.loginfo(tag + " Mean: v {:.6f}, w = {:.6f}.".format(np.mean(lin), np.mean(ang)))
        rospy.loginfo(tag + " Std: v {:.6f}, w = {:.6f}.".format(np.std(lin), np.std(ang)))
        rospy.loginfo(tag + " Var: v {:.6f}, w = {:.6f}.\n".format(np.var(lin), np.var(ang)))

    def show_graphs(self, lin_vels, ang_vels, controls):
        path_rng = np.arange(len(lin_vels))
        plt.figure(1)
        plt.subplot(221)
        plt.plot(path_rng, lin_vels)
        plt.yscale("linear")
        plt.title("Linear velocitie")
        plt.xlabel("point")
        plt.ylabel("Linear vel")

        plt.subplot(222)
        plt.plot(path_rng, ang_vels)
        plt.yscale("linear")
        plt.title("Angular")
        plt.xlabel("point")
        plt.ylabel("Angular vel")

        control_len = len(controls)
        control_rng = np.arange(control_len)
        lin_controls = controls[:, 0]
        ang_controls = controls[:, 1]

        plt.subplot(223)
        plt.plot(control_rng, lin_controls)
        plt.yscale("linear")
        plt.title("Linear Control")
        plt.xlabel("point")
        plt.ylabel("Linear vel")

        plt.subplot(224)
        plt.plot(control_rng, ang_controls)
        plt.yscale("linear")
        plt.title("Angular Control")
        plt.xlabel("point")
        plt.ylabel("Angular vel")
        plt.show()
