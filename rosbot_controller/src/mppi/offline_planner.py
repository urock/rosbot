#!/usr/bin/env python3
import numpy as np
from time import time
import nnio

# from typing import Type
import argparse

from optimizers.pso import PSO
from utils.dtypes import State


"""
    Offline planner generates optimal control sequence that should move a robot 
    to its goal taking into account constraints (static obstacles). 

    Должен ли он генерить последовательность управления на фиксированное время вперед?

    Может быть надо брать длину последовательности управления заведомо большую и ожидать, 
    что крайние точки будут нулевыми.

    based on https://en.wikipedia.org/wiki/Particle_swarm_optimization

"""


class OfflinePlanner:
     
     def __init__(self, model_path, obstacles = None):

        self.batch_size = 100
        self.time_steps = 1000
        self.n_iters = 10
        self.state_size = 5     # size of state vector X
        self.control_size = 2   # size of state vector U
        self.model = nnio.ONNXModel(model_path)

        self.optimizer = PSO(self.batch_size, self.time_steps, self.control_size)

        self.dt = 0.033



    def run(self, current_state, goal, constraints=None):
        """
            
        """

        batch_u = self.optimizer.init_control_batch()
        batch_x = self._propagate_control_to_states(current_state, batch_u)
        batch_costs = self._calculate_costs(batch_x, constraints)

        for it in range(self.n_iters):

            batch_u = self.optimizer.gen_next_control_batch(batch_costs)            
            batch_x = self._propagate_control_to_states(current_state, batch_u)
            batch_costs = self._calculate_costs(batch_x, goal)

        best_contol = self.optimizer.get_best_control() 

        return best_contol
        

    def _propagate_control_to_states(self, current_state, batch_u):
        """ 
            Calulates batch of sequences of states based on inputs:
            Inputs: current state, batch of control sequencies
            Return: batch of state sequencies

        """ 
        # batch of sequences of robot states
        batch_x = np.empty(shape=(self.batch_size, self.time_steps, self.state_size))
        batch_x[:,0] = current_state

        # 5 for v, w, control_dim and dt
        model_inp_vector_size = 5
        batch_model_input_seqs = np.zeros(shape=(self.batch_size, self.time_steps, model_inp_vector_size))
        self.batch_model_input_seqs[:, :, 4] = self.dt

        # TODO continue here 
        # also rework _predict_trajectories and implement _calculate_costs

        

        return batch_x

    def _predict_trajectories(self):
        """ Propagetes trajectories using control matrix velocities and current state

        Return:
            trajectory points - np.array of shape [batch_size, time_steps, 3] where 3 is for x, y, yaw respectively
        """

        v, w = self.batch_of_seqs[:, :, 0], self.batch_of_seqs[:, :, 1]
        current_yaw = self.curr_state.yaw
        yaw = np.cumsum(w * self.dt, axis=1)
        yaw += current_yaw - yaw[:, :1]
        v_x = v * np.cos(yaw)
        v_y = v * np.sin(yaw)
        x = np.cumsum(v_x * self.dt, axis=1)
        y = np.cumsum(v_y * self.dt, axis=1)
        x += self.curr_state.x - x[:, :1]
        y += self.curr_state.y - y[:, :1]

        traj_points = np.concatenate([
            x[:, :, np.newaxis],
            y[:, :, np.newaxis],
            yaw[:, :, np.newaxis],
        ], axis=2)
        return traj_points


    def _calculate_costs(self, batch_x, goal):
        """
            Calculates batch of costs (cost for each predicted seqeunce of robot states)
            For now takes into account only final goal
        """

        batch_costs = np.zeros(shape=(self.batch_size))

        return batch_costs


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-m', '--model_path', type=str, required=True,
                        help='Path to nn model file')

    args = parser.parse_args()

    current_state = np.zeros(5)
    goal = (1.0, 1.0)   # (x, y)

    planner = OfflinePlanner(args.model_path)
    control = planner.run(current_state, goal)
    print(control)


    
if __name__ == '__main__':
    main()