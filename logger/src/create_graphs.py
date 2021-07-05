#!/usr/bin/env python
import os
# import seaborn as sns
import pandas as pd
import numpy as np
import argparse
import matplotlib.pyplot as plt
from logger.logger_tools import plot_xy_data, save_plot

"""
python3 
for one folder with data
python3 create_graphs.py -folder_path /home/user/catkin_ws/src/logger/output_data/test_PSO_2_3_150_dt_0.03/test_PSO_2_3_150_dt_0.03_1

or
for many folders
python3 create_graphs.py -folder_path /home/user/catkin_ws/src/logger/output_data/test_PSO_2_3_150_dt_0.03 -group True

"""

def parse_file(folder_path, file, data):
    """ """

    pwd = os.getcwd()
    os.chdir(folder_path)
    path = folder_path + '/' + file
    data['name'] = file[0:-4]
    with open(path, 'r') as f:
        keys = f.readline()
        keys = keys.rstrip().split(' ')  # get keys from first line, remove '/n'
        if "#" in keys:
            keys.remove("#")
        for line in f.readlines():
            line = line[0:-1].split(' ')
            for i in range(len(keys)):
                data[keys[i]].append(float(line[i]))

    os.chdir(pwd)

def parse_one_trajectory(folder_path):

    # declare file names with data
    required_files = ['state.csv', 'kinetic_model_state.csv', 'control.csv', 'time.csv']

    # declare containers for data
    robot_state = {'x': [], 'y': [], 'yaw': [], 'v': [], 'w': []}
    model_state = {'x': [], 'y': [], 'yaw': [], 'v': [], 'w': []}
    control = {'x': [], 'yaw': []}
    time = {'t': []}
    # print(time)
    # create container for all containers to simplify work with them

    # parse each file and  fill the containers
    for data, file in zip([robot_state, model_state, control, time], required_files):
        parse_file(folder_path, file, data)

    try:
        nn_model_state = {'x': [], 'y': [], 'yaw': [], 'v': [], 'w': []}
        parse_file(folder_path, "nn_model_state.csv", nn_model_state)
    except:
        nn_model_state = None
        print("There are no files with the state of the neural network model!")

    return robot_state, model_state, control, time, nn_model_state


def plot_for_one_trajectory(args, folder_path):

    robot_state, model_state, control, time, nn_model_state = parse_one_trajectory(folder_path)

    # print(robot_state)
    # time = np.cumsum(np.array(delta_time['dt']))
    fig, ax = plt.subplots(2)
    ax[0].set_ylabel('m/s')
    ax[1].set_ylabel('m/s')
    ax[1].set_xlabel('t, sec')
    ax[0].set_title("linear velocity and control")
    plot_xy_data(x=time['t'], y=robot_state['v'], ax=ax[0], plot_name="robot v")
    plot_xy_data(x=time['t'], y=control['x'], ax=ax[0], plot_name="u1")
    if len(model_state['v']) > 0:
        plot_xy_data(x=time['t'], y=model_state['v'], ax=ax[0], plot_name="kinematic model v")
    if nn_model_state is not None or len(nn_model_state['v']) >0:
        plot_xy_data(x=time['t'], y=nn_model_state['v'], ax=ax[0], plot_name="nn model v")

    ax[1].set_title("angular velocity and control")
    plot_xy_data(x=time['t'], y=robot_state['w'], ax=ax[1], plot_name="robot w")
    plot_xy_data(x=time['t'], y=control['yaw'], ax=ax[1], plot_name="u2")
    if len(model_state['w']) > 0:
        plot_xy_data(x=time['t'], y=model_state['w'], ax=ax[1], plot_name="kinematic model w")
    if nn_model_state is not None or len(nn_model_state['w']) > 0:
        plot_xy_data(x=time['t'], y=nn_model_state['w'], ax=ax[1], plot_name="nn model w")


    if args.output_folder != "":
        save_plot(path=args.output_folder, name="velocities_and_control")    

    # plot X(t), Y(t)
    fig2, ax2 = plt.subplots(2)
    ax2[0].set_ylabel('m')
    ax2[1].set_ylabel('m')
    ax2[1].set_xlabel('t, sec')    
    ax2[0].set_title("X coord over time")
    plot_xy_data(x=time['t'], y=robot_state['x'], ax=ax2[0], plot_name="robot x(t)")
    plot_xy_data(x=time['t'], y=model_state['x'], ax=ax2[0], plot_name="kinematic model x(t)")
    if nn_model_state is not None:
        plot_xy_data(x=time['t'], y=nn_model_state['x'], ax=ax2[0], plot_name="nn model x(t)")

    ax2[1].set_title("Y coord over time")
    plot_xy_data(x=time['t'], y=robot_state['y'], ax=ax2[1], plot_name="robot y(t)")
    plot_xy_data(x=time['t'], y=model_state['y'], ax=ax2[1], plot_name="kinematic model y(t)")
    if nn_model_state is not None:
        plot_xy_data(x=time['t'], y=nn_model_state['y'], ax=ax2[1], plot_name="nn model y(t)")

    if args.output_folder != "":
        save_plot(path=args.output_folder, name="XY over time")        

    # plot yaw(t)
    fig3, ax3 = plt.subplots(1)
    ax3.set_xlabel('t, sec')        
    ax3.set_ylabel('Rads')
    ax3.set_title("Yaw angle over time")
    plot_xy_data(x=time['t'], y=robot_state['yaw'], ax=ax3, plot_name="yaw(t)")
    plot_xy_data(x=time['t'], y=model_state['yaw'], ax=ax3, plot_name="kinematic model yaw(t)")
    if nn_model_state is not None:
        plot_xy_data(x=time['t'], y=nn_model_state['yaw'], ax=ax3, plot_name="nn model yaw(t)")

    if args.output_folder != "":
        save_plot(path=args.output_folder, name="YAW over time")        

    # plot Y(X)
    fig4, ax4 = plt.subplots(1)
    ax4.set_xlabel('X, m')        
    ax4.set_ylabel('Y, m')
    ax4.set_title("XY trajectory")
    plot_xy_data(x=robot_state['x'], y=robot_state['y'], ax=ax4, plot_name="ground truth")
    plot_xy_data(x=model_state['x'], y=model_state['y'], ax=ax4, plot_name="kinematic model")
    if nn_model_state is not None:
        plot_xy_data(x=nn_model_state['x'], y=nn_model_state['y'], ax=ax4, plot_name="NN model")
    
    if args.output_folder != "":
        save_plot(path=args.output_folder, name="Y over X")        

    plt.show()

def plot_for_group(args, folder_path):
    """
    # FOR PSO planner
    """
    fig, ax = plt.subplots(1)
    ax.set_xlabel('X, m')        
    ax.set_ylabel('Y, m')
    ax.set_title("XY trajectory")

    robot_state = {'x': [], 'y': [], 'yaw': [], 'v': [], 'w': []}
    nn_model_state = {'x': [], 'y': [], 'yaw': [], 'v': [], 'w': []}
    model_state = {'x': [], 'y': [], 'yaw': [], 'v': [], 'w': []}
    control = {'x': [], 'yaw': []}
    time = {'t': []}

    robot_state = pd.DataFrame(robot_state, columns=robot_state.keys())
    model_state = pd.DataFrame(model_state, columns=robot_state.keys())
    nn_model_state = pd.DataFrame(nn_model_state, columns=robot_state.keys())

    for traj in os.listdir(folder_path):
        traj_path = folder_path + '/' + traj
        if not os.path.isdir(traj_path):
            continue
        robot_state_, model_state_, control_, time_, nn_model_state_ = parse_one_trajectory(traj_path)
        robot_state_ = pd.DataFrame(robot_state_, columns=robot_state.keys())
        model_state_ = pd.DataFrame(model_state_, columns=robot_state.keys())
        nn_model_state_ = pd.DataFrame(nn_model_state_, columns=robot_state.keys())
      
        robot_state_.plot(x='x', y='y', ax=ax, legend=False, c='b', alpha=0.2, kind='line')
        model_state_.plot(x='x', y='y', ax=ax, legend=False, c='r', alpha=0.2, kind='line')
        nn_model_state_.plot(x='x', y='y', ax=ax, grid=True, legend=False, c='g', alpha=0.2, kind='line')

    ax.legend(['ground truth', 'kinematic model', 'NN model' ])
    save_plot(folder_path, name="complex plot")
    plt.show()

def main():
    """ """

    plt.rcParams['font.size'] = '12'

    parser = argparse.ArgumentParser()

    parser.add_argument('-group', action='store', default=False, dest='group',
                        required=False, help='')

    parser.add_argument('-folder_path', action='store', dest='folder_path',
                        required=True, help='absolute path to the folder with data.csv')
    
    parser.add_argument('-output_folder', action='store', dest='output_folder', 
                        required=False, default="", 
                        help="absolute path to the output folder to store images. No images are stored if empty")                        
    args = parser.parse_args()
    folder_path = args.folder_path

    if not args.group:
        plot_for_one_trajectory(args, folder_path)
    else:
        plot_for_group(args, folder_path)
    



if __name__ == "__main__":
    main()
