import numpy as np
import os
import matplotlib.pyplot as plt

def visualize_trajectory(x, ax):
    """
        Args:
            x: np.array of shape (time_steps, state_size)
                    state is [x, y, yaw, v, w]
    """
    ax.set_xlabel('X, m')        
    ax.set_ylabel('Y, m')
    # ax.set_title("Best XY trajectory")
    ax.set_aspect(1)
    plot_xy_data(x=x[:,0], y=x[:,1], ax=ax, plot_name="Optimal trajectory")

    # plt.show()

def visualize_costs(batch_costs, ax):
    """
        Args:
            batch_costs: np.array of shape (batch_size)
    """
    ax.set_title("Current batch of costs")
    ax.set_xlabel('batch_idx')        
    ax.set_ylabel('cost')

    plot_xy_data(x=[i for i in range(len(batch_costs))], y=batch_costs, ax=ax, plot_name="costs")

    # plt.show()

def visualize_control(u, dt, ax1, ax2, plot_name):
    """
        Args:
            u: np.array of shape (time_steps, control_size)
    """
    # ax1.set_title("Control graph")
    ax1.set_xlabel('t, s')        
    ax1.set_ylabel('linear velocity')
    ax2.set_ylabel('angular velocity')

    t = [i*dt for i in range(u.shape[0])]

    plot_xy_data(x=t, y=u[:,0], ax=ax1, plot_name=plot_name)
    plot_xy_data(x=t, y=u[:,1], ax=ax2, plot_name=plot_name)


def plot_xy_data(x, y, ax=None, plot_name=" "):
    """Build a graph from x and y"""
    x = np.array(x)
    y = np.array(y)
    if ax == None:
        plt.plot(x, y, marker='o', label=plot_name)
        # plt.plot(x, y, label=plot_name)
        plt.grid(True)
        # plt.title(title)
    else:
        ax.plot(x, y, marker='o', label=plot_name)
        # ax.plot(x, y, label=plot_name)
        ax.grid(True)
        # ax.set_title(title)
        # ax.annotate(
        #     plot_name,
        #     xy=(0, 1), xytext=(12, -12), va='top',
        #     xycoords='axes fraction', textcoords='offset points',
        #     bbox=dict(facecolor='none', edgecolor='black')
        # )
        ax.legend(loc="best")


def plot_data(data, ax=None, plot_name=" "):
    """Build a graph from x or y """
    if ax == None:
        plt.plot(data, label=plot_name)
        plt.grid(True)
        # plt.title(plot_name)
    else:
        ax.plot(data, label=plot_name)
        ax.grid(True)
        # ax.set_title(plot_name)
        ax.legend(loc="best")
        # ax.annotate(
        #     plot_name,
        #     xy=(0, 1), xytext=(12, -12), va='top',
        #     xycoords='axes fraction', textcoords='offset points',
        #     bbox=dict(facecolor='none', edgecolor='black')
        # )


def show_graph():
    """ """

    try:
        plt.show()
        plt.close()
    except:
        pass


def save_plot(path, name='', fmt='png'):
    """Saves graph to the output pkg"""

    pwd = os.getcwd()
    os.chdir(os.path.expanduser('~'))
    if not os.path.exists(path):
        os.makedirs(path)
    os.chdir(path)
    plt.savefig('{}.{}'.format(name, fmt), fmt='png')
    os.chdir(pwd)


def write_to_file(path, data, file_name):
    """Saves data to the output file"""

    pwd = os.getcwd()
    os.chdir(os.path.expanduser('~'))
    if not os.path.exists(path):
        os.makedirs(path)

    output_file = open(path + file_name + '.csv', 'w')
    keys = ('dt', 't', 'x', 'y', 'yaw', 'v', 'w')
    # write title
    title = ''
    for key in keys:
        if key in data.keys():
            title += key + ' '
    output_file.write(title + '\n')

    for i in range(0, len(data.values()[0])):
        item = str()
        for key in keys:
            if key in data.keys():
                item = item + str(round(data[key][i], 5)) + ' '
        output_file.write(item.strip() + '\n')


    output_file.close()
    os.chdir(pwd)
