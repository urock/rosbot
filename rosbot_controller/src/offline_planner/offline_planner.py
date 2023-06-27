#!/usr/bin/env python3
import os
import numpy as np
from time import time
from time import perf_counter
import nnio
import argparse
import matplotlib.pyplot as plt

from modules.pso import PSO

from modules.plot_tools import plot_xy_data
from modules.plot_tools import visualize_trajectory, visualize_control, visualize_costs 


"""
    Offline planner generates optimal control sequence that should move a robot 
    to its goal taking into account constraints (static obstacles). 

    Должен ли он генерить последовательность управления на фиксированное время вперед?

    Может быть надо брать длину последовательности управления заведомо большую и ожидать, 
    что крайние точки будут нулевыми.

    based on https://en.wikipedia.org/wiki/Particle_swarm_optimization

"""



class OfflinePlanner:
     
    def __init__(self, n_iters, model_path, output_path):

        self.batch_size = 20000

        # self.dt = 0.033          # sec
        self.dt = 0.03          # sec
        self.pso_dt = 1         # sec. PSO outputs control with pso_dt interval between samples 
        self.total_time = 15    # sec 
        self.pso_steps              = int(self.total_time/ self.pso_dt) + 1
        self.time_steps             = int(self.total_time / self.dt)
        self.num_dt_for_pso_step    = int(self.pso_dt/ self.dt)

        self.v_max, self.w_max = 1, 1 
        self.v_min = 0.0

        self.goal_tolerance = 0.01
        self.Tmax = 15.0

        self.n_iters = n_iters
        self.output_path = output_path
        self.state_size = 5     # size of state vector X
        self.control_size = 2   # size of state vector U
        
        self.use_nn_model = False

        if model_path is not None:
            self.model = nnio.ONNXModel(model_path)
            self.use_nn_model = True

        self.optimizer = PSO(self.batch_size, self.pso_steps, self.control_size, self.v_max, self.w_max)

        
    def run(self, current_state, goal, obstacles):

        print("Run started")

        batch_pso = self.optimizer.init_control_batch()
        batch_x = self._propagate_control_to_states(current_state, batch_pso)
        idx_min, t_min, dist_min, trajectory_length = self._calculate_costs(batch_x, goal, obstacles)
        batch_costs = t_min + dist_min + trajectory_length # добавить веса
        
        self.optimizer.update_bests(batch_costs)

        print("Run: init ok, Best Batch Cost = {:.10f}".format(np.min(batch_costs)))

        start = perf_counter() 

        for it in range(self.n_iters):

            start = perf_counter() 

            batch_pso = self.optimizer.gen_next_control_batch()            
            batch_x = self._propagate_control_to_states(current_state, batch_pso)

            idx_min, t_min, dist_min, trajectory_length = self._calculate_costs(batch_x, goal, obstacles)
            batch_costs = t_min + dist_min + trajectory_length

            self.optimizer.update_bests(batch_costs)

            t = perf_counter() 
          
            best_pso, best_cost = self.optimizer.get_best_control() 
            best_x = self._propagate_control_to_states(current_state, best_pso[None])
            idx_min, t_min, dist_min, trajectory_length = self._calculate_costs(best_x, goal, obstacles)

            print("Run: {} its done. dt = {:.3f} s. Best Cost = {:.10f}. t_min = {:.2f} dist_min = {:.2f} trajectory_length = {:.2f}".
                        format(it, t - start, best_cost, t_min[0], dist_min[0], trajectory_length[0]))            

            
        best_pso, best_cost = self.optimizer.get_best_control() 
        best_x = self._propagate_control_to_states(current_state, best_pso[None])
        best_u = self._expand_control(best_pso[None])

        idx_min, t_min, dist_min, trajectory_length = self._calculate_costs(best_x, goal, obstacles)

        reaching_goal_idx = idx_min[0]

        x = best_x[0][reaching_goal_idx, 0]
        y = best_x[0][reaching_goal_idx, 1]
        yaw = best_x[0][reaching_goal_idx, 2]    

        print("Last point ({:.4f},{:.4f}, {:.4f}) reached @ {:.2f} sec with Cost = {:.10f}".
                format(x, y, yaw, reaching_goal_idx*self.dt, best_cost))

        pso_control = best_pso[:int(reaching_goal_idx/self.num_dt_for_pso_step) + 1]
        final_control = best_u[0][:reaching_goal_idx + 1]
        final_trajectory = best_x[0][:reaching_goal_idx + 1]   

        self._plot_graphs(pso_control, final_control, final_trajectory, obstacles)
        # self._plot_graphs(pso_control, best_u[0], best_x[0], obstacles)

        if self.output_path is not None:
            self.save_control_to_csv(final_control, self.output_path)

        return final_control


    def _plot_graphs(self, pso_control, final_control, final_trajectory, obstacles):
        
        fig1, ax1 = plt.subplots(1)
        visualize_trajectory(final_trajectory, ax1)

        for obstacle in obstacles:
            circle1 = plt.Circle((obstacle[0], obstacle[1]), obstacle[2], color='r')
            ax1.add_patch(circle1)

        fig2, ax2 = plt.subplots(2)
        visualize_control(final_control, self.dt, ax2[0], ax2[1], "Best Control")
        visualize_control(pso_control, self.pso_dt, ax2[0], ax2[1], "PSO Control")
        
        fig3, ax3 = plt.subplots(3)

        ax3[0].set_title("State over time")
        ax3[0].set_xlabel('t, s')        

        t = [i*self.dt for i in range(final_control.shape[0])]

        plot_xy_data(x=t, y=final_trajectory[:, 0], ax=ax3[0], plot_name="X")
        plot_xy_data(x=t, y=final_trajectory[:, 1], ax=ax3[1], plot_name="Y")        
        plot_xy_data(x=t, y=final_trajectory[:, 2], ax=ax3[2], plot_name="Yaw")        
        
        plt.show()
        

    
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

        batch_v_pso = batch_pso[:,:,0]
        batch_w_pso = batch_pso[:,:,1]

        m = self.num_dt_for_pso_step

        for i in range(num_pso_steps):       

            # in coord system with center in i's pso step
            k_v = (batch_v_pso[:,i+1] - batch_v_pso[:,i])/self.pso_dt
            k_w = (batch_w_pso[:,i+1] - batch_w_pso[:,i])/self.pso_dt
            
            # for current line (k, b) on this interation I have to fill 
            # batch_u[:, i*num_dt_for_pso_step:(i+1)*num_dt_for_pso_step]
            for j in range(m):
                batch_temp_v = batch_v_pso[:,i] + k_v[:]*j*self.dt
                batch_temp_w = batch_w_pso[:,i] + k_w[:]*j*self.dt

                # control threshold
                mask = batch_temp_v > self.v_max
                batch_temp_v[mask] = self.v_max
                mask = batch_temp_v < self.v_min
                batch_temp_v[mask] = self.v_min   

                mask = batch_temp_w > self.w_max
                batch_temp_w[mask] = self.w_max
                mask = batch_temp_w < -self.w_max
                batch_temp_w[mask] = -self.w_max 

                batch_u[:,i*m + j,0] = batch_temp_v
                batch_u[:,i*m + j,1] = batch_temp_w


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

        if self.use_nn_model:
            batch_v, batch_w = self._inference_nn_model(current_state, batch_u)
        else:
            batch_v, batch_w = batch_u[:,:,0], batch_u[:,:,1]

        batch_trajectories = self._calc_trajectories(current_state, batch_v, batch_w, self.dt)

        batch_x[:,:,:3] = batch_trajectories
        batch_x[:,:,3] = batch_v
        batch_x[:,:,4] = batch_w

        return batch_x


    def _inference_nn_model(self, current_state, batch_u):
        """ 
            Calulates batch of sequences of velocities:
            Args: 
                current state: np.array of shape (state_size) [x, y, yaw, v, w]
                batch_u: np.array of shape (batch_size, time_steps, control_size)
            Return: 
                batch_v: np.array of shape (batch_size, time_steps) - linear velocity
                batch_w: np.array of shape (batch_size, time_steps) - anglular velocity
        """ 
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
        for t_step in range(self.time_steps - 1):
            curr_batch = batch_model_input_seqs[:, t_step].astype(np.float32)
            curr_predicted = self.model(curr_batch)
            batch_model_input_seqs[:, t_step + 1, :2] = curr_predicted            

        t = perf_counter() 
        print("Inference time: dt = {:.3f} s".format(t - start))            

        batch_v = batch_model_input_seqs[:, :, 0]
        batch_w = batch_model_input_seqs[:, :, 1]

        return batch_v, batch_w    
                

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


    def _calculate_costs(self, batch_x, goal, obstacles):
        """
            Calculates batch of costs (cost for each predicted seqeunce of robot states)
            For now takes into account only final goal
        """

        return self._cost_for_goal_and_time(batch_x, goal, obstacles)


    def _cost_for_goal_and_time(self, batch_x, goal, obstacles):
        """
        Args:
            batch_x: np.array of shape (batch_size, time_steps, state_size) 
            goal (list of 2 elements): coord of main goal 
        Return:
            idx_min: batch of indexes at which goal has been reached
            batch_costs: batch of times reaching the goal np.array of shape (batch_size)
        """
        t_min = np.empty(batch_x.shape[0])
        idx_min = np.empty(batch_x.shape[0], int)
        dist_min = np.empty(batch_x.shape[0])


        # calc dist to goal
        x = batch_x[:, :, 0]    # take every point
        y = batch_x[:, :, 1]

        L2 = ((x-goal[0])**2 + (y-goal[1])**2) * 50

        # check if goal is reached in Tmax time
        # if reached - save reaching time
        for i in range(L2.shape[0]):    # batch index loop
            closest_to_goal_points = np.argwhere(L2[i] < self.goal_tolerance) # [T]
            if closest_to_goal_points.shape[0] != 0:
                idx_min[i] = int(closest_to_goal_points[0])
                t_min[i] = self.dt*idx_min[i]
                if t_min[i] > self.Tmax:
                    t_min[i] = self.Tmax
            else:
                t_min[i] = self.Tmax
                idx_min[i] = int(L2.shape[1] - 1)
            
            # find min distance to goal 
            dist_min[i] = np.min(L2[i,:idx_min[i] + 1])

        # 

        # calc trajectory_length
        dx = x[:,1:] - x[:,:-1] 
        dy = y[:,1:] - y[:,:-1] 
        dr2 = np.sqrt((dx)**2 + (dy)**2) # [B, T]
        time_idx = np.arange(dr2.shape[1])[None].repeat(dr2.shape[0], 0) # [B, T]
        mask = time_idx <= idx_min[:, None] # [B, T]
    
        trajectory_length = np.sum(dr2, axis=1)
        trajectory_length_m = np.sum(dr2 * mask, axis=1)

        print("{:.4f}, {:.4f}".
                format(trajectory_length[0], trajectory_length_m[0]))

        # cost for obstacles
        obstacle_cost = np.zeros(L2.shape[0])
        for obstacle in obstacles:
            center_x, center_y = obstacle[0], obstacle[1]
            radius = obstacle[2]
            
            obstacle_dist2 = (x - center_x)**2 + (y - center_y)**2      # shape = (batch_size, time_steps)

            obstacle_mask = np.min(obstacle_dist2,axis=1) < radius**2   # shape = (batch_size)

            obstacle_cost[obstacle_mask] += 100
        return idx_min, t_min, dist_min, trajectory_length + obstacle_cost
        # return idx_min, t_min + dist_min + trajectory_length + obstacle_cost
        # return idx_min, t_min, dist_min, trajectory_length_m
        
    def save_control_to_csv(self, control_seq, output_path):
        """
        Saves the control sequence in сsv format.
        Args:
            :control_seq: (): control sequence
            :output_path: (str): path where the file will be saved

        """
        #print(type(control_seq))
        #print(control_seq)
        parent_dir, _ = os.path.split(output_path)
        if not os.path.exists(parent_dir):
            os.makedirs(parent_dir)

        with open(output_path, 'w') as f:
            f.write("t x yaw \n")
            t = 0.0
            for item in control_seq:
                f.write(str(t) + " " + str(item[0]) + " " + str(item[1]) + "\n")
                t = t + self.dt

        print("Control file path = {}".format(os.path.abspath(output_path)))



def main():


    SMALL_SIZE = 8
    MEDIUM_SIZE = 10
    BIGGER_SIZE = 20

    plt.rc('font', size=BIGGER_SIZE)          # controls default text sizes
    plt.rc('axes', titlesize=BIGGER_SIZE)     # fontsize of the axes title
    plt.rc('axes', labelsize=BIGGER_SIZE)    # fontsize of the x and y labels
    plt.rc('xtick', labelsize=BIGGER_SIZE)    # fontsize of the tick labels
    plt.rc('ytick', labelsize=BIGGER_SIZE)    # fontsize of the tick labels
    plt.rc('legend', fontsize=MEDIUM_SIZE)    # legend fontsize
    plt.rc('figure', titlesize=BIGGER_SIZE)  # fontsize of the figure title

    parser = argparse.ArgumentParser()
    parser.add_argument('-m', '--model_path', type=str, required=False,
                        help='Path to nn onnx model')
    parser.add_argument('-it', '--n_iters', type=int, required=False, default=10,
                        help='Path to nn onnx model')   
    parser.add_argument('-o', '--output_path', type=str, required=False, default=None,
                        help='Path to the output file with control')                      

    args = parser.parse_args()

    current_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0])   # (x, y, yaw, v, w)
    goal = [2.0, 4.0]                                     # (x, y)

    # center x, center y, radius
    obstacles = [(0.5, 0.5, 0.35), (2.0, 2.0, 0.35), (2.0, 3.5, 0.35), (0.4, 2, 0.35), (0, 3, 0.35)]           

    planner = OfflinePlanner(args.n_iters, args.model_path, args.output_path)
    control = planner.run(current_state, goal, obstacles)
    # control = planner.test(current_state, goal)
    
if __name__ == '__main__':
    main()

 

