import os
import math

class TrajectoryData():
    """ class for data 
        text
        text
        text"""
    
    def __init__(self):  
        
        
        self.data_ = {'t': [], 'x': [], 'y': [], 'yaw': [],
                           'u_v': [], 'u_w': [],  'base_x': [], 'base_y': [], 'base_yaw': []}


    def from_dir(self, dir_path):
        """  t, x, y, yaw, u_v, u_w, base_x, base_y, base_yaw """

        # Parsing data from files

        robot_state_fpath = dir_path + '/robot_state.txt'
        with open(robot_state_fpath) as f:
            for line in f.readlines()[1:]:
                line = line.rstrip().split(' ')
                self.data_['t'].append(float(line[0]))
                self.data_['x'].append(float(line[1]))
                self.data_['y'].append(float(line[2]))
                self.data_['yaw'].append(float(line[3]))       
        
        control_fpath = dir_path + '/control.txt'
        with open(control_fpath) as f:
            for line in f.readlines()[1:]:
                line = line.rstrip().split(' ')
                self.data_['u_v'].append(float(line[1]))
                self.data_['u_w'].append(float(line[2]))
        

        self.data_['base_x'].append(self.data_['x'][0])
        self.data_['base_y'].append(self.data_['y'][0])
        self.data_['base_yaw'].append(self.data_['yaw'][0])
        for i in range(len(self.data_['t'])-1):
            x, y, yaw = self.update_state_by_model(self.data_['base_x'][i], 
                                                   self.data_['base_y'][i],
                                                   self.data_['base_yaw'][i],
                                                   self.data_['u_v'][i],
                                                   self.data_['u_w'][i],
                                                   self.data_['yaw'][i], 
                                                   self.data_['t'][i+1] - self.data_['t'][i])
            
            
            self.data_['base_x'].append(x)
            self.data_['base_y'].append(y)
            self.data_['base_yaw'].append(yaw)
           
          
        


    def update_state_by_model(self, state_x, state_y, state_yaw, v, w, yaw, dt):  
        """
        Updates robot state assuming that control has been active for dt seconds
        c : control vector of RobotControl type
        dt : time period in seconds
        """


        if abs(w) > 0.001:
            rho = v / w

            # step 1. Calc new robot position relative to its previous pose
            x_r = rho * math.sin(w*dt)
            y_r = rho * (1 - math.cos(w*dt))

            # step 2. Transfrom this point to map fixed coordinate system taking into account current robot pose
            state_x += x_r * math.cos(yaw) - y_r * math.sin(yaw)
            state_y += x_r * math.sin(yaw) + y_r * math.cos(yaw)
            state_yaw +=  w * dt
        else:
            state_x += v * dt * math.cos(yaw)
            state_y += v * dt * math.sin(yaw)
            state_yaw = None

        return state_x, state_y, state_yaw