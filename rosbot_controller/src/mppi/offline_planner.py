#!/usr/bin/env python3
import numpy as np
from time import time
from time import perf_counter
import nnio

# from typing import Type
import argparse

from optimizers.pso import PSO
from utils.dtypes import State

import matplotlib.pyplot as plt
from utils.logger_tools import plot_xy_data, save_plot
from utils.logger_tools import visualize_trajectory, visualize_control, visualize_costs 


"""
    Offline planner generates optimal control sequence that should move a robot 
    to its goal taking into account constraints (static obstacles). 

    Должен ли он генерить последовательность управления на фиксированное время вперед?

    Может быть надо брать длину последовательности управления заведомо большую и ожидать, 
    что крайние точки будут нулевыми.

    based on https://en.wikipedia.org/wiki/Particle_swarm_optimization

"""



class OfflinePlanner:
     
    def __init__(self, model_path_1, model_path_100, obstacles = None):

        self.batch_size = 100

        self.dt = 0.05          # sec
        self.pso_dt = 1         # sec. PSO outputs control with pso_dt interval between samples 
        self.total_time = 10    # sec 
        self.pso_steps              = int(self.total_time/ self.pso_dt) + 1
        self.time_steps             = int(self.total_time / self.dt)
        self.num_dt_for_pso_step    = int(self.pso_dt/ self.dt)

        self.v_max, self.w_max = 1.0, 1.0 

        self.n_iters = 20
        self.state_size = 5     # size of state vector X
        self.control_size = 2   # size of state vector U
        self.model_1 = nnio.ONNXModel(model_path_1)
        self.model_100 = nnio.ONNXModel(model_path_100)

        self.optimizer = PSO(self.batch_size, self.pso_steps, self.control_size, self.v_max, self.w_max)

        
    def run(self, current_state, goal):

        print("Run started")

        batch_pso = self.optimizer.init_control_batch()
        batch_x = self._propagate_control_to_states_no_nn(current_state, batch_pso)
        idx_min, batch_costs = self._calculate_costs(batch_x, goal)
        
        self.optimizer.update_bests(batch_costs)

        print("Run: init ok")

        start = perf_counter() 

        for it in range(self.n_iters):

            start = perf_counter() 

            batch_pso = self.optimizer.gen_next_control_batch()            
            batch_x = self._propagate_control_to_states_no_nn(current_state, batch_pso)
            idx_min, batch_costs = self._calculate_costs(batch_x, goal)
            self.optimizer.update_bests(batch_costs)

            t = perf_counter() 
          
            best_pso, best_cost = self.optimizer.get_best_control() 

            print("Run: {} iterartions done. dt = {:.3f} s. Cost = {:.10f}".
                        format(it, t - start, best_cost))            

            
        best_pso, best_cost = self.optimizer.get_best_control() 
        print("batch_pso.shape = " + str(best_pso.shape))
        print("best_cost.shape = " + str(best_cost.shape))

        best_x = self._propagate_control_to_states_no_nn(current_state, best_pso[None])
        print("best_x.shape = " + str(best_x.shape))

        best_u = self._expand_control(best_pso[None])
        print("best_u.shape = " + str(best_u.shape))


        idx_min, batch_costs = self._calculate_costs(best_x, goal)

        print(idx_min)

        reaching_goal_idx = idx_min[0]

        x = best_x[0][reaching_goal_idx, 0]    # take only last element
        y = best_x[0][reaching_goal_idx, 1]
        yaw = best_x[0][reaching_goal_idx, 2]    

        print("Last point ({:.4f},{:.4f}, {:.4f}) Cost = {:.10f}".format(x, y, yaw, best_cost))

        final_control = best_u[0][:reaching_goal_idx]
        print("final_control.shape = " + str(final_control.shape))


        fig1, ax1 = plt.subplots(1)
        visualize_trajectory(best_x[0][:reaching_goal_idx], ax1)
        fig2, ax2 = plt.subplots(2)
        visualize_control(final_control, self.dt, ax2[0], ax2[1], "Best Control")
        
        fig3, ax3 = plt.subplots(3)

        ax3[0].set_title("State over time")
        ax3[0].set_xlabel('t, s')        

        t = [i*self.dt for i in range(reaching_goal_idx + 1)]

        plot_xy_data(x=t, y=best_x[0][:reaching_goal_idx + 1, 0], ax=ax3[0], plot_name="X")
        plot_xy_data(x=t, y=best_x[0][:reaching_goal_idx + 1, 1], ax=ax3[1], plot_name="Y")        
        plot_xy_data(x=t, y=best_x[0][:reaching_goal_idx + 1, 2], ax=ax3[2], plot_name="Yaw")        

        plt.show()   

        return final_control

    
    def _expand_control(self, batch_pso):
        """ 
            PSO generates control with self.pso_dt (1 sec) time step. This function
            fills control sequence with self.dt (50 ms) time step. 
            Args: 
                batch_pso: np.array of shape (batch_size, pso_steps, control_size)
            Return: 
                batch_u: np.array of shape (batch_size, time_steps, control_size)
        """ 
        batch_u = np.empty(shape=(batch_pso.shape[0], self.time_steps, self.control_size))

        num_pso_steps = batch_pso.shape[1] - 1

        # batch_v_pso = batch_pso[:,:,0]
        # batch_w_pso = batch_pso[:,:,1]

        m = self.num_dt_for_pso_step

        for i in range(num_pso_steps):       

            # in coord system with center in i's pso step
            # k_v = (batch_v_pso[:,i+1] - batch_v_pso[:,i])/self.pso_dt
            # k_w = (batch_w_pso[:,i+1] - batch_w_pso[:,i])/self.pso_dt
            
            k = (batch_pso[:,i+1] - batch_pso[:,i])/self.pso_dt

            #TODO merge k_v with k_w (add last dim)

            # for current line (k, b) on this interation I have to fill 
            # batch_u[:, i*num_dt_for_pso_step:(i+1)*num_dt_for_pso_step]

            for j in range(m):
                # batch_u[:,i*m + j,0] = batch_v_pso[:,i] + k_v[:]*j*self.dt
                # batch_u[:,i*m + j,1] = batch_w_pso[:,i] + k_w[:]*j*self.dt
                batch_u[:,i*m + j] = batch_pso[:,i] + k[:]*j*self.dt

        return batch_u
        

    def _propagate_control_to_states(self, current_state, batch_pso):
        """ 
            Calulates batch of sequences of states based on inputs:
            Args: 
                current state: np.array of shape (state_size) [x, y, yaw, v, w]
                batch_pso: np.array of shape (batch_size, pso_steps, control_size)
            Return: 
                batch_x: np.array of shape (batch_size, time_steps, state_size)

        """ 
        batch_u = self._expand_control(batch_pso)
        # batch of sequences of robot states
        batch_x = np.empty(shape=(batch_u.shape[0], self.time_steps, self.state_size))

        # 5 for v, w, control_dim and dt
        model_inp_vector_size = 5
        batch_model_input_seqs = np.zeros(shape=(batch_u.shape[0], self.time_steps, model_inp_vector_size))
        
        # fill current velosities to time step 0
        batch_model_input_seqs[:, 0, 0] = current_state[3]     # v
        batch_model_input_seqs[:, 0, 1] = current_state[4]     # w
        # fill control and dt to all time steps
        batch_model_input_seqs[:, :, 2:4] = batch_u
        batch_model_input_seqs[:, :, 4] = self.dt

        start = perf_counter() 

        # predict velocities
        if batch_u.shape[0] == 100:
            for t_step in range(self.time_steps - 1):
                curr_batch = batch_model_input_seqs[:, t_step].astype(np.float32)
                curr_predicted = self.model_100(curr_batch)
                batch_model_input_seqs[:, t_step + 1, :2] = curr_predicted
        else:
            for t_step in range(self.time_steps - 1):
                curr_batch = batch_model_input_seqs[:, t_step].astype(np.float32)
                curr_predicted = self.model_1(curr_batch)
                batch_model_input_seqs[:, t_step + 1, :2] = curr_predicted            

        t = perf_counter() 
        # print("Inference time: dt = {:.3f} s".format(t - start))            

        batch_v = batch_model_input_seqs[:, :, 0]
        batch_w = batch_model_input_seqs[:, :, 1]
        batch_trajectories = self._calc_trajectories(current_state, batch_v, batch_w, self.dt)

        batch_x[:,:,:3] = batch_trajectories
        batch_x[:,:,3] = batch_v
        batch_x[:,:,4] = batch_w

        return batch_x


    def _propagate_control_to_states_no_nn(self, current_state, batch_pso):
        """ 
            Calulates batch of sequences of states based on inputs:
            Args: 
                current state: np.array of shape (state_size) [x, y, yaw, v, w]
                batch_pso: np.array of shape (batch_size, pso_steps, control_size)
            Return: 
                batch_x: np.array of shape (batch_size, time_steps, state_size)

        """ 
        batch_u = self._expand_control(batch_pso)
        # print("batch_u.shape = " + str(batch_u.shape))


        # fig1, ax1 = plt.subplots(2)
        # visualize_control(batch_pso[0], self.pso_dt, ax1[0], ax1[1], "PSO Control")

        # fig2, ax2 = plt.subplots(2)
        # visualize_control(batch_u[0], self.dt, ax2[0], ax2[1], "Expanded Control")

        # plt.show()  

        # batch of sequences of robot states
        batch_x = np.empty(shape=(batch_u.shape[0], self.time_steps, self.state_size))
          

        batch_v = batch_u[:,:,0]
        batch_w = batch_u[:,:,1]
        batch_trajectories = self._calc_trajectories(current_state, batch_v, batch_w, self.dt)

        batch_x[:,:,:3] = batch_trajectories
        batch_x[:,:,3] = batch_v
        batch_x[:,:,4] = batch_w

        # print("batch_x.shape = " + str(batch_x.shape))
    
        return batch_x
                

    def _calc_trajectories(self, current_state, batch_v, batch_w, dt):
        """ Propagetes trajectories using control matrix velocities and current state

            current state: np.array of shape (state_size) [x, y, yaw, v, w]
            batch_v: linear vels, np.array of shape (batch_size, time_steps, 1)
            batch_w: angular vels, np.array of shape (batch_size, time_steps, 1)

        Return:
            trajectory points - np.array of shape (batch_size, time_steps, 3) 
        """

        v, w = batch_v, batch_w
        current_yaw = current_state[2]
        yaw = np.cumsum(w * dt, axis=1)
        yaw += current_yaw - yaw[:, :1]
        v_x = v * np.cos(yaw)
        v_y = v * np.sin(yaw)
        x = np.cumsum(v_x * dt, axis=1)
        y = np.cumsum(v_y * dt, axis=1)
        x += current_state[0] - x[:, :1]
        y += current_state[1] - y[:, :1]

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

        return self._cost_for_goal_and_time(batch_x, goal)


    def _cost_for_goal_and_time(self, batch_x, goal):
        """
        Args:
            batch_x: np.array of shape (batch_size, time_steps, state_size) 
            goal (list of 2 elements): coord of main goal 
        Return:
            batch_costs: batch of times reaching the goal np.array of shape (batch_size)
        """

        eps = 0.01
        Tmax = 10.0

        x = batch_x[:, :, 0]    # take every point
        y = batch_x[:, :, 1]

        L2 = (x-goal[0])**2 + (y-goal[1])**2
        
        # print("L2.shape = " + str(L2.shape))

        t_min = np.empty(L2.shape[0])
        idx_min = np.empty(L2.shape[0], int)
        dist_min = np.empty(L2.shape[0])

        # check if goal is reached in Tmax time
        # if reached - save reaching time
        for i in range(L2.shape[0]):    # batch index loop
            aw = np.argwhere(L2[i] < eps)
            if aw.shape[0] != 0:
                idx_min[i] = int(np.argwhere(L2[i] < eps)[0])
                t_min[i] = self.dt*(np.argwhere(L2[i] < eps)[0])
                if t_min[i] > Tmax:
                    t_min[i] = Tmax
            else:
                t_min[i] = Tmax
                idx_min[i] = int(L2.shape[1] - 1)
            
            # find min distance to goal 
            dist_min[i] = np.min(L2[i,:idx_min[i] + 1])

        dx = x[:,1:] - x[:,:-1] 
        dy = y[:,1:] - y[:,:-1] 
        dr2 = (dx)**2 + (dy)**2

        return idx_min, t_min + dist_min + np.sum(dr2, axis=1)
        
           


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-m100', '--model_path_100', type=str, required=True,
                        help='Path to nn model file with batch size 100')
    parser.add_argument('-m1', '--model_path_1', type=str, required=True,
                        help='Path to nn model file with batch size 1')

    args = parser.parse_args()

    current_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0])   # (x, y, yaw, v, w)
    goal = [2.0, 2.0]                                   # (x, y)

    planner = OfflinePlanner(args.model_path_1, args.model_path_100)
    control = planner.run(current_state, goal)
    # control = planner.test(current_state, goal)
    
if __name__ == '__main__':
    main()



    # def test(self, current_state, goal):

    #     print("Test started")

    #     u = np.zeros(shape=(self.time_steps, self.control_size))
    #     u[:,0] = 0.1
    #     u[:,1] = 0.2

    #     # x = self._propagate_control_to_states_no_nn(current_state, u[None])
    #     x = self._propagate_control_to_states(current_state, u[None])

    #     fig, ax = plt.subplots(3)
    #     visualize_trajectory(x, ax[0])
    #     visualize_control(u, self.dt, ax[1], ax[2])
        
    #     plt.show()      


    # def _cost_for_goal(self, batch_x, goal):
    #     """
    #     Args:
    #         batch_x: np.array of shape (batch_size, time_steps, state_size) 
    #         goal (list of 2 elements): coord of main goal 
    #     Return:
    #         batch_costs: batch_x: np.array of shape (batch_size)
    #     """


    #     x = batch_x[:, self.time_steps-1, 0]    # take only last element
    #     y = batch_x[:, self.time_steps-1, 1]    



    #     L2 = (x-goal[0])**2 + (y-goal[1])**2

    #     x = batch_x[:, 0::10, 0]    # take every 10th element
    #     y = batch_x[:, 0::10, 1]    


    #     delta_x = x[:, 1:] - x[:, :-1]
    #     delta_y = y[:, 1:] - y[:, :-1]


    #     dl = delta_x**2 + delta_y**2

    #     dl_sum = np.sum(dl, axis=1)

    #     dl_y = np.abs(np.sum(y, axis=1))
    #     # dl_sum = np.concatenate(dl_sum, 0)

    #     # return np.sum(L2, axis=1)
    #     # return L2 + 0.1 * np.sum(v,axis=1)
    #     return L2
    #     # return L2 + 0.1 * dl_sum
    #     # return L2 + dl_y + dl_sum    