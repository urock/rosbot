#!/usr/bin/env python
import argparse
import matplotlib.pyplot as plt
import numpy as np
import sys
import os


def parse_data(global_path, data, file_path):
    try:
        i = 0
        for traj in os.listdir(global_path):
            j = 0
            path = global_path + '/' + traj + file_path
            with open(path) as f:
                f.readline()
                for line in f.readlines():
                    line = line.split() 
                    data[i, j, 0] = float(line[1]) # x
                    data[i, j, 1] = float(line[2]) # y
                    j = j +1
                    if j >= data.shape[1]:
                        break  
            i = i + 1
    except Exception as e:
        print(e)

def main():
    """

    """
    global_path = sys.argv[1]
    traj_num = int(sys.argv[2])

    trajectories = np.zeros([traj_num, 1000, 2]) # shape = [batch, time, x_y]
    parse_data(global_path=global_path, data=trajectories, file_path='/data/robot_state.txt')
    mean_traj = np.mean(trajectories[:, :, :], axis=0)

    model_trajectories = np.zeros([traj_num, 1000, 2]) # shape = [batch, time, x_y]
    parse_data(global_path=global_path, data=model_trajectories, file_path='/data/model_state.txt')
    model_mean_traj = np.mean(model_trajectories[:, :, :], axis=0)
    

    nn_trajectory = np.zeros([1, 1000, 2]) # shape = [batch, time, x_y]
    # i = 0
    # j = 0
    # file_path = "/home/vytautas/Desktop/nn_model_state.txt"
    # with open(file_path) as f:
    #     f.readline()
    #     for line in f.readlines():
    #         line = line.split() 
    #         nn_trajectory[i, j, 0] = float(line[1]) # x
    #         nn_trajectory[i, j, 1] = float(line[2]) # y
    #         j = j +1
    #         if j >= 1000:
    #             break  


    fig, ax = plt.subplots(1)
    # add NN model state
    ax.set_title('Sharing Y axis')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.plot(
        nn_trajectory[0, :, 0],
        nn_trajectory[0, :, 1],
        label='NN model',
        color='black',
        ls='--'
    )
    # add mean robot state
    ax.plot(
        mean_traj[:, 0],
        mean_traj[:, 1],
        label='rosbot',
        color='red',
        ls='--'
    )
    # add all robot states
    for traj in trajectories:        
        ax.plot(
            traj[:, 0],
            traj[:, 1],
            color='blue',
            alpha=0.2
        );
    # add mean kinetic model state
    ax.plot(
        model_mean_traj[:, 0],
        model_mean_traj[:, 1],
        label='kinecmatic model',
        color='orange',
        ls='--'
    )
    # add all kinetic model state
    for traj in model_trajectories:        
        ax.plot(
            traj[:, 0],
            traj[:, 1],
            color='orange',
            alpha=0.2
        );
    ax.grid(ls='--', alpha=0.2)
    ax.legend();

    plt.show()


if __name__ == "__main__":
    main()
