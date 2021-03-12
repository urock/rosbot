#!/usr/bin/env python
# license removed for brevity
import os
import numpy as np
import pandas as pd
import argparse


def main():
    """Convert .txt files with robot state and control
     to pretty csv table for ML"""

    parser = argparse.ArgumentParser()
    parser.add_argument('-folder_path', action='store', dest='folder_path',
                        required=True, help='absolute path to the folder with data')

    path = parser.parse_args().folder_path

    sud_dframes = []

    for dir in os.listdir(path):
        # print(dir)

        dir_path = path + '/' + dir + '/data'
        robot_state_file_path = dir_path + '/robot_state.txt'
        control_file_path = dir_path + '/control.txt'

        sub_data = {'time_ctrl': [], 'x_prev': [0], 'y_prev': [0], 'yaw_prev': [0], 'x_cur': [], 'y_cur': [], 'yaw_cur': [], 'lin_control': [],
                'ang_control': [], 'x_next': [], 'y_next': [], 'yaw_next': []}

        # parse control file
        with open(control_file_path) as f:
            keys = f.readline()
            keys = ['time_ctrl', 'lin_control', 'ang_control']
            for line in f.readlines():
                line = line[0:-1].split(' ')
                for i in range(len(keys)):
                    sub_data[keys[i]].append(float(line[i]))

        # parse robot_state file
        with open(robot_state_file_path) as f:
            keys = f.readline()
            # keys = ['x_prev', 'y_prev', 'yaw_prev', 'x_cur', 'y_cur', 'yaw_cur', 'x_next', 'y_next', 'yaw_next']
            all_lines = f.readlines()
            for i in range(1,len(all_lines)-1):
                
                # print(all_lines[i-1].split(' ')[0:-1], all_lines[i].split(' ')[0:-1], all_lines[i+1].split(' ')[0:-1])

                sub_data['x_prev'].append(all_lines[i-1].split(' ')[0:-1][1])
                sub_data['y_prev'].append(all_lines[i -1].split(' ')[0:-1][2])
                sub_data['yaw_prev'].append(all_lines[i -1].split(' ')[0:-1][2])

                sub_data['x_cur'].append(all_lines[i].split(' ')[0:-1][1])
                sub_data['y_cur'].append(all_lines[i].split(' ')[0:-1][2])
                sub_data['yaw_cur'].append(all_lines[i].split(' ')[0:-1][2])

                sub_data['x_next'].append(all_lines[i+1].split(' ')[0:-1][1])
                sub_data['y_next'].append(all_lines[i+1].split(' ')[0:-1][2])
                sub_data['yaw_next'].append(all_lines[i+1].split(' ')[0:-1][2])

                
        sub_df = pd.DataFrame(sub_data, columns=['time_ctrl', 'lin_control', 'ang_control'])

        sud_dframes.append(sub_df)

    df = pd.concat(sud_dframes)
    df.to_csv('~/test.csv', index=False)


if __name__ == "__main__":
    main()
