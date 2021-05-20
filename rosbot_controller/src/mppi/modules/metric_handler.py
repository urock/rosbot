import rospy
import numpy as np
import matplotlib.pyplot as plt


class MetricHandler():
    def __init__(self, metric):
        self.metric = metric
        self.path = np.zeros(shape=(0, 5))  # x, y, yaw, v, w
        self.controls = np.zeros(shape=(0, 2))  # v w

        self.exec_times = []

    def add_exec_time(self, t):
        self.exec_times.append(t)

    def get_mean_time(self):
        return np.sum(self.exec_times) / len(self.exec_times)

    def get_std_time(self):
        return np.std(self.exec_times)

    def reset(self):
        self.path = np.zeros(shape=(0, 5))  # x, y, yaw, v, w
        self.controls = np.zeros(shape=(0, 2))  # v w
        self.exec_times = []

    def add_state(self, state):
        self.path = np.append(self.path, state.to_numpy()[np.newaxis], axis=0)

    def add_control(self, control):
        self.controls = np.append(self.controls, control.to_numpy()[np.newaxis], axis=0)

    def show_metrics(self, time, reference_trajectory):
        lin_vels = self.path[:, 3]
        ang_vels = self.path[:, 4]

        lin_controls = self.controls[:, 0]
        ang_controls = self.controls[:, 1]

        value = self.metric(reference_trajectory, self.path)
        rospy.loginfo("Path Total Time: {:.6f}.".format(time))
        rospy.loginfo("Path Error by {}: {:.6f}.\n".format(self.metric.__name__, value))

        self.show_statistics('Vels', lin_vels, ang_vels)
        self.show_statistics('Controls', lin_controls, ang_controls)

        # self.plot_vels(lin_vels, ang_vels, self.controls)
        # self.plot_trajs(self.path, reference_trajectory)
        # plt.show()

    def show_statistics(self, tag, lin, ang):
        rospy.loginfo(tag + " Mean: v {:.6f}, w = {:.6f}.".format(np.mean(lin), np.mean(ang)))
        rospy.loginfo(tag + " Std: v {:.6f}, w = {:.6f}.".format(np.std(lin), np.std(ang)))
        rospy.loginfo(tag + " Var: v {:.6f}, w = {:.6f}.\n".format(np.var(lin), np.var(ang)))

    def plot_trajs(self, path, reference):
        plt.figure(2)
        plt.plot(path[:, 0], path[:, 1], label='Path')
        plt.plot(reference[:, 0], reference[:, 1], marker='d', ms=2.0, label='Trajectory')

        plt.yscale("linear")
        plt.title("Trajectory")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.legend()

    def plot_vels(self, lin_vels, ang_vels, controls):
        lin_rng = np.arange(len(lin_vels))
        plt.figure(1)
        plt.subplot(221)
        plt.plot(lin_rng, lin_vels)
        plt.yscale("linear")
        plt.title("Linear velocity")
        plt.xlabel("point")
        plt.ylabel("Linear vel")

        ang_rng = np.arange(len(lin_vels))
        plt.subplot(222)
        plt.plot(ang_rng, ang_vels)
        plt.yscale("linear")
        plt.title("Angular velocity")
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
