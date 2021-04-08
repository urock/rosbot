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

    # plot velocities and control
    fig, ax = plt.subplots(2)
    ax[0].set_ylabel('m/s')
    ax[1].set_ylabel('m/s')
    ax[1].set_xlabel('t, sec')
    ax[0].set_title("linear velocity and control")
    plot_xy_data(x=time, y=robot_state['v'], ax=ax[0], plot_name="robot v")
    plot_xy_data(x=time, y=model_state['v'], ax=ax[0], plot_name="kinematic model v")
    plot_xy_data(x=time, y=control['x'], ax=ax[0], plot_name="u1")

    ax[1].set_title("angular velocity and control")
    plot_xy_data(x=time, y=robot_state['w'], ax=ax[1], plot_name="robot w")
    plot_xy_data(x=time, y=model_state['w'], ax=ax[1], plot_name="kinematic model w")
    plot_xy_data(x=time, y=control['yaw'], ax=ax[1], plot_name="u2")

    # plot X(t), Y(t)
    fig2, ax2 = plt.subplots(2)
    ax2[0].set_ylabel('m')
    ax2[1].set_ylabel('m')
    ax2[1].set_xlabel('t, sec')    
    ax2[0].set_title("X coord over time")
    plot_xy_data(x=time, y=robot_state['x'], ax=ax2[0], plot_name="robot x(t)")
    plot_xy_data(x=time, y=model_state['x'], ax=ax2[0], plot_name="kinematic model x(t)")
    
    ax2[1].set_title("Y coord over time")
    plot_xy_data(x=time, y=robot_state['y'], ax=ax2[1], plot_name="robot y(t)")
    plot_xy_data(x=time, y=model_state['y'], ax=ax2[1], plot_name="kinematic model y(t)")

    # plot yaw(t)
    fig3, ax3 = plt.subplots(1)
    ax3.set_xlabel('t, sec')        
    ax3.set_ylabel('Rads')
    ax3.set_title("Yaw angle over time")
    plot_xy_data(x=time, y=robot_state['yaw'], ax=ax3, plot_name="yaw(t)")
    plot_xy_data(x=time, y=model_state['yaw'], ax=ax3, plot_name="kinematic model yaw(t)")

    # plot Y(X)
    fig4, ax4 = plt.subplots(1)
    ax4.set_xlabel('X, m')        
    ax4.set_ylabel('Y, m')
    ax4.set_title("XY trajectory")
    plot_xy_data(x=robot_state['x'], y=robot_state['y'], ax=ax4, plot_name="x_y")
    plot_xy_data(x=model_state['x'], y=model_state['y'], ax=ax4, plot_name="kinematic model x_y")

    plt.show()
    # path = self.module_path + '/pictures'
    # save_plot(path=path, name="velocities_and_control")


if __name__ == "__main__":
    main()
