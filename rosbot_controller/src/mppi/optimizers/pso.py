#!/usr/bin/env python3
import numpy as np


class PSO:

    def __init__(self, batch_size=100, time_steps=100, dim_size=2, 
                v_max=1.0, w_max=1.0,
                w=0.8, c1=1, c2=1, learing_rate=0.5):

        self.batch_size = batch_size
        self.time_steps = time_steps
        self.dim_size = dim_size   # size of state vector U

        # current batch of controls = batch of current particles position   
        self.batch_u = np.zeros(shape=(self.batch_size, self.time_steps, self.dim_size))
        # batch of current particles velocities   
        self.batch_v = np.zeros(shape=(self.batch_size, self.time_steps, self.dim_size))

        # best batch of controls = batch of best particles position
        self.batch_p = np.zeros(shape=(self.batch_size, self.time_steps, self.dim_size))

        # best position of best particle
        self.g = np.zeros(shape=(self.time_steps, self.dim_size))  

        self.best_costs = np.full(self.batch_size, np.inf)
        self.global_best = np.inf      

        self.w, self.c1, self.c2, self.lr = w, c1, c2, learing_rate
        self.v_max, self.w_max = v_max, w_max 
        

    
    def init_control_batch(self):
        """
            Initilasize batch of controls and returns it
        """
        self.batch_u[:, :, 0] = np.random.uniform(0, self.v_max, (self.batch_size, self.time_steps, 1))
        self.batch_u[:, :, 1] = np.random.uniform(-self.w_max, self.w_max, (self.batch_size, self.time_steps, 1))

        self.batch_p = self.batch_u

        return self.batch_u


    def gen_next_control_batch(self, batch_costs):
        """
            Generate next control (particle positions) batch based on current particles position, 
            best position of each particle and best position of all particles
        """

        self.update_bests(batch_costs)

        # update particles velocities
        p_r = np.random.uniform(size=(self.batch_size, self.time_steps, 2))
        g_r = np.random.uniform(size=(self.batch_size, self.time_steps, 2))
        
        self.batch_v = self.w * self.batch_v 
        self.batch_v += self.c1 * p_r * (self.batch_p - self.batch_u)
        self.batch_v += self.c2 * g_r * (self.g[np.newaxis] - self.batch_u)

        self.batch_u += self.lr * self.batch_v

        return self.batch_u


    def update_bests(self, costs):
        """
            1. For each particle (batch index) select best position, 
            2. select best position of all particles
            3. Save best positions to global class memory
        """

        mask = costs < self.best_costs
        self.best_costs[mask] = costs[mask]
        self.batch_p[mask] = self.batch_x[mask]
        self.global_best = self.batch_p[np.argmin(self.best_costs)] 