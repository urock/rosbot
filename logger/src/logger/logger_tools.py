import numpy as np
import os
import matplotlib.pyplot as plt


def plot_xy_data(x, y, ax=None, plot_name=" "):
    """Build a graph from x and y"""
    x = np.array(x)
    y = np.array(y)
    if ax == None:
        plt.plot(x, y, label=plot_name)
        plt.grid(True)
        plt.title(plot_name)
    else:
        ax.plot(x, y, label=plot_name)
        ax.grid(True)
        ax.annotate(
            plot_name,
            xy=(0, 1), xytext=(12, -12), va='top',
            xycoords='axes fraction', textcoords='offset points',
            bbox=dict(facecolor='none', edgecolor='black')
        )


def plot_data(data, ax=None, plot_name=" "):
    """Build a graph from x or y """
    if ax == None:
        plt.plot(data, label=plot_name)
        plt.grid(True)
        plt.title(plot_name)
    else:
        ax.plot(data, label=plot_name)
        ax.grid(True)
        ax.annotate(
            plot_name,
            xy=(0, 1), xytext=(12, -12), va='top',
            xycoords='axes fraction', textcoords='offset points',
            bbox=dict(facecolor='none', edgecolor='black')
        )


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
    keys = ('dt', 'x', 'y', 'yaw', 'v', 'w')
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
