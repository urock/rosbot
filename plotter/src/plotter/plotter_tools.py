import numpy as np
import os
import matplotlib.pyplot as plt


def plot_xy_data(x, y):
    """Build a graph from x and y"""
    x = np.array(x)
    y = np.array(y)
    plt.plot(x, y)
    plt.grid(True)


def plot_data(data):
    """Build a graph from x or y """

    plt.plot(data)
    plt.grid(True)


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

    output_file = open(path + file_name + '.txt', 'w')

    for i in range(0, len(data.values()[0])):
        item = str()
        for key in ('t', 'x', 'y', 'yaw'):
            if key in data.keys():
                item = item + str(round(data[key][i], 2)) + ' '
        output_file.write(str(item) + '\n')

    output_file.close()
    os.chdir(pwd)
