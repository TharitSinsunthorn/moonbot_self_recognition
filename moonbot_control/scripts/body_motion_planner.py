#! /usr/bin/env python3

import numpy as np
import math
import time


class BodyMotionPlanner():
    '''
    Class for body motion control
    '''
    def __init__(self , cmd, leg, body, gait_planner):
        self.cmd = cmd
        self.body = body
        self.leg = leg
        self.gait = gait_planner

        self.prev_slant = self.cmd.body.slant

    def set_init_pose(self):
        self.cmd.leg.foot_zero_pnt[:,2] = np.array(self.cmd.body.height) 
            
        for i in range(4):
            self.cmd.leg.foot_zero_pnt[i,0] = 0.18 * np.cos((2*i+1)*math.pi/4) #+ self.cmd.body.slant[0]
            self.cmd.leg.foot_zero_pnt[i,1] = 0.18 * np.sin((2*i+1)*math.pi/4) #+ self.cmd.body.slant[1]
            
        self.body.roll = self.cmd.body.roll
        self.body.pitch = self.cmd.body.pitch
        self.body.yaw = self.cmd.body.yaw
    
    def run(self):
        while True:
            
            # Iniitialize the pose
            self.set_init_pose()
            
            # Define the legs and their respective trajectory attributes
            legs = ['RF', 'LF', 'LR', 'RR']
            # trajectories = [self.gait.RF_traj, self.gait.LF_traj, self.gait.LR_traj, self.gait.RR_traj]
            trajectories = self.gait.traj

            # Iterate over each leg and assign the values
            for i, leg in enumerate(legs):
                cur_coord = np.array(self.cmd.leg.foot_zero_pnt[i, :]) \
                                    + trajectories[leg] \
                                    + self.body.ZMP_handler[i, :] * np.array([1, 1, 0]) \
                                    + self.cmd.body.slant * np.array([1, 1, 0])
                                    
                getattr(self.leg, leg).pose.cur_coord[:] = cur_coord

            time.sleep(0.0002)