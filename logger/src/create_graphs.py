#!/usr/bin/env python
import os
import numpy as np
import argparse
import matplotlib.pyplot as plt
from logger.logger_tools import plot_xy_data

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

    os.chdir(pwd)


# def build_graph(data, folder_path):
#     """ """

#     # TODO make a universal function
#     x1, y1 = np.array(data[0]['x']), np.array(data[0]['y'])
#     x2, y2 = np.array(data[1]['x']), np.array(data[1]['y'])
#     x3, y3 = np.array(data[2]['x']), np.array(data[2]['y'])

#     plt.rcParams.update({'font.size': 14})  # font size
#     plt.rcParams['figure.figsize'] = (11.0, 8.0)
#     plt.figure("trajectory and states")

#     plt.plot(x1, y1, color='b', label='robot state', linewidth=3)
#     plt.plot(x2, y2, color='r', label='model state', linewidth=3)
#     plt.plot(x3, y3, color='g', label='trajectory', linewidth=3)

#     plt.xlabel('X')
#     plt.ylabel('Y')
#     plt.legend(loc='best')  # or loc=1
#     plt.grid(True)
#     save_plot(folder_path, name='graph', fmt='png')
#     plt.show()


def main():
    """ """
    FONT_SIZE = '12'
    plt.rcParams['font.size'] = '12'
    # parse folder path from command line
    parser = argparse.ArgumentParser()
    parser.add_argument('-folder_path', action='store', dest='folder_path',
                        required=True, help='absolute path to the folder with data.csv')
    args = parser.parse_args()
    folder_path = args.folder_path

    # declare file names with data
    required_files = ['state.csv', 'kinetic_model_state.csv', 'control.csv', 'delta_time.csv']

    # declare containers for data
    robot_state = {'x': [], 'y': [], 'yaw': [], 'v': [], 'w': []}
    model_state = {'x': [], 'y': [], 'yaw': [], 'v': [], 'w': []}
    control = {'x': [], 'yaw': []}
    delta_time = {'dt': []}
    # print(time)
    # create container for all containers to simplify work with them

    # parse each file and  fill the containers
    for data, file in zip([robot_state, model_state, control, delta_time], required_files):
        parse_file(folder_path, file, data)

    # print(robot_state)
    time = np.cumsum(np.array(delta_time['dt']))
    fig, ax = plt.subplots(2)
    plot_xy_data(x=time, y=control['x'], ax=ax[0], plot_name="u1")
    plot_xy_data(x=time, y=robot_state['v'], ax=ax[0], plot_name="robot v")
    plot_xy_data(x=time, y=model_state['v'], ax=ax[0], plot_name="kinematic model v")

    plot_xy_data(x=time, y=control['yaw'], ax=ax[1], plot_name="u2")
    plot_xy_data(x=time, y=robot_state['w'], ax=ax[1], plot_name="robot w")
    plot_xy_data(x=time, y=model_state['w'], ax=ax[1], plot_name="kinematic model w")

    fig2, ax2 = plt.subplots(2)
    plot_xy_data(x=time, y=robot_state['x'], ax=ax2[0], plot_name="robot x(t)")
    plot_xy_data(x=time, y=model_state['x'], ax=ax2[0], plot_name="kinematic model x(t)")
    plot_xy_data(x=time, y=robot_state['y'], ax=ax2[1], plot_name="robot y(t)")
    plot_xy_data(x=time, y=model_state['y'], ax=ax2[1], plot_name="kinematic model y(t)")


    fig3, ax3 = plt.subplots(1)
    plot_xy_data(x=time, y=robot_state['yaw'], ax=ax3, plot_name="yaw(t)")
    plot_xy_data(x=time, y=model_state['yaw'], ax=ax3, plot_name="kinematic model yaw(t)")

    fig4, ax4 = plt.subplots(1)
    plot_xy_data(x=robot_state['x'], y=robot_state['y'], ax=ax4, plot_name="x_y")
    plot_xy_data(x=model_state['x'], y=model_state['y'], ax=ax4, plot_name="kinematic model x_y")

    # fig2.annotate(
    #     "states",
    #     xy=(0, 1), xytext=(12, -12), va='top',
    #     xycoords='axes fraction', textcoords='offset points',
    #     bbox=dict(facecolor='none', edgecolor='black')
    # )


    plt.show()
    # path = self.module_path + '/pictures'
    # save_plot(path=path, name="velocities_and_control")


if __name__ == "__main__":
    main()
