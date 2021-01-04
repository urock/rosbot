#!/usr/bin/env python
import os
import numpy as np
import argparse
import matplotlib.pyplot as plt
from plotter.plotter_tools import save_plot


def get_txt_files(path):
    """Get list of all .txt files from path"""

    pwd = os.getcwd()
    os.chdir(path)
    txt_files = list()

    for f in os.listdir(path):
        if f.endswith(".txt"):
            txt_files.append(f)
    os.chdir(pwd)
    return txt_files


def parse_file(folder_path, file, data):
    """ """

    pwd = os.getcwd()
    os.chdir(folder_path)
    path = folder_path + '/' + file
    data['name'] = file[0:-4]
    with open(path, 'r') as f:
        keys = f.readline()
        keys = keys.rstrip().split(' ')  # get keys from first line, remove '/n'
        for line in f.readlines():
            line = line[0:-1].split(' ')
            for i in range(len(keys)):
                data[keys[i]].append(float(line[i]))

    # print(data['name'])
    # print(type(data['y'][0]))
    # print(data)
    os.chdir(pwd)


def build_graph(data, folder_path):
    """ """

    # TODO make a universal function
    x1, y1 = np.array(data[0]['x']), np.array(data[0]['y'])
    x2, y2 = np.array(data[1]['x']), np.array(data[1]['y'])
    x3, y3 = np.array(data[2]['x']), np.array(data[2]['y'])

    plt.rcParams.update({'font.size': 14})  # font size
    plt.rcParams['figure.figsize'] = (11.0, 8.0)
    plt.figure("trajectory and states")

    plt.plot(x1, y1, color='b', label='robot state', linewidth=3)
    plt.plot(x2, y2, color='r', label='model state', linewidth=3)
    plt.plot(x3, y3, color='g', label='trajectory', linewidth=3)

    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend(loc='best')  # or loc=1
    plt.grid(True)
    save_plot(folder_path, name='graph', fmt='png')
    plt.show()


def main():
    """ """

    # parse folder path from command line
    parser = argparse.ArgumentParser()
    parser.add_argument('-folder_path', action='store', dest='folder_path',
                        required=True, help='absolute path to the folder with data.txt')
    args = parser.parse_args()
    folder_path = args.folder_path

    # declare file names with data
    required_files = ['robot_state.txt', 'model_state.txt', 'trajectory.txt', 'control.txt']

    # declare containers for data
    robot_state = {'t': [], 'x': [], 'y': [], 'yaw': []}
    model_state = {'t': [], 'x': [], 'y': [], 'yaw': []}
    trajectory = {'x': [], 'y': []}
    # create container for all containers to simplify work with them
    all_data = [robot_state, model_state, trajectory]

    # get list of txt files from folder
    txt_files = get_txt_files(folder_path)

    # parse each file and  fill the containers
    for data, file in zip(all_data, required_files):
        parse_file(folder_path, file, data)

    build_graph(all_data, folder_path)


if __name__ == "__main__":
    main()
