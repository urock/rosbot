#!/usr/bin/env python3
import numpy as np
from time import time

from typing import Type

from optimizers.pso import PSO



"""
    Offline planner generates optimal control sequence that should move a robot 
    to its goal taking into account constraints (static obstacles). 

    Должен ли он генерить последовательность управления на фиксированное время вперед?

    Может быть надо брать длину последовательности управления заведомо большую и ожидать, 
    что крайние точки будут нулевыми.

    based on https://en.wikipedia.org/wiki/Particle_swarm_optimization

"""


class OfflinePlanner:
     
     def __init__(self, goal, obstacles):

        self.batch_size = 100
        self.time_steps = 1000
        self.n_iters = 10
        self.state_size = 5     # size of state vector X
        self.control_size = 2   # size of state vector U

        


    def run(self, current_state, goal, constraints):

        self._init_controls()

        for it in range(self.n_iters):
            
            batch_x = self._propagate_control_to_states(current_state, self.batch_u)
            batch_costs = self._calculate_costs(batch_x, constraints)
            self._pick_best_particles(self.batch_u, batch_costs)
            self._gen_next_control_batch()
        




    def _propagate_control_to_states(self, current_state, batch_u):
        """ 
            Calulates batch of sequences of states based on inputs:
            Inputs: current state, batch of control sequencies
            Return: batch of state sequencies

        """ 
        batch_x = np.zeros(shape=(self.batch_size, self.time_steps, self.state_size))

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


    def _calculate_costs(self, batch_x, constraints):

        batch_costs = np.zeros(shape=(self.batch_size))

        return batch_costs


    
    