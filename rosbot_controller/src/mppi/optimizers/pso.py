#!/usr/bin/env python3
import numpy as np


class PSO:

    def __init__(self, batch_size=100, time_steps=10, control_size=2, 
                v_max=1.0, w_max=1.0,
                w=0.8, c1=0.15, c2=0.85, learing_rate=1.0):

        self.batch_size = batch_size
        self.time_steps = time_steps
        self.control_size = control_size   # size of state vector U

        # current batch of controls = batch of current particles position   
        self.batch_u = np.zeros(shape=(self.batch_size, self.time_steps, self.control_size))
        # batch of current particles velocities   
        self.batch_v = np.zeros(shape=(self.batch_size, self.time_steps, self.control_size))

        # best batch of controls = batch of best particles position
        self.batch_p = np.zeros(shape=(self.batch_size, self.time_steps, self.control_size))

        # best position of best particle
        self.g = np.zeros(shape=(self.time_steps, self.control_size))  

        self.best_costs = np.full(self.batch_size, np.inf)
        self.global_best = np.inf      

        self.w, self.c1, self.c2, self.lr = w, c1, c2, learing_rate
        self.v_max, self.w_max = v_max, w_max 
        

    
    def init_control_batch(self, first_run=False):
        """
            Initilasize batch of controls

            Return: np.array of shape (batch_size, time_steps, control_size) 
        """

        self.batch_u[:, :, 0] = np.random.uniform(0, self.v_max, (self.batch_size, self.time_steps))
        self.batch_u[:, :, 1] = np.random.uniform(-self.w_max, self.w_max, (self.batch_size, self.time_steps))
      
        self.batch_v[:, :, 0] = np.random.uniform(-self.v_max, self.v_max, (self.batch_size, self.time_steps))
        self.batch_v[:, :, 1] = np.random.uniform(-2*self.w_max, 2*self.w_max, (self.batch_size, self.time_steps))

        self.batch_p = self.batch_u

        # print("batch_pso.shape = " + str(self.batch_u.shape))

        return self.batch_u


    def gen_next_control_batch(self):
        """
            Generate next control (particle positions) batch based on current particles position, 
            best position of each particle and best position of all particles

            Return: np.array of shape (batch_size, time_steps, control_size)
        """

        # update particles velocities
        p_r = np.random.uniform(size=(self.batch_size, self.time_steps, 2))
        g_r = np.random.uniform(size=(self.batch_size, self.time_steps, 2))
        
        self.batch_v = self.w * self.batch_v 
        self.batch_v += self.c1 * p_r * (self.batch_p - self.batch_u)
        self.batch_v += self.c2 * g_r * (self.g[np.newaxis] - self.batch_u)

        self.batch_u += self.lr * self.batch_v

        # print("batch_pso.shape = " + str(self.batch_u.shape))
        return self.batch_u


    def get_best_control(self):
        return self.global_best, np.min(self.best_costs)


    def update_bests(self, costs):
        """
            1. For each particle (batch index) select best position, 
            2. select best position of all particles
            3. Save best positions to global class memory
        """

        mask = costs < self.best_costs

        self.best_costs[mask] = costs[mask]

        mask = mask[:, None, None]
        self.batch_p = self.batch_u * mask + self.batch_p * (1 - mask)
        self.global_best = self.batch_p[np.argmin(self.best_costs)]
