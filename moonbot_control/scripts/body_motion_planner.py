#! /usr/bin/env python3

import numpy as np
import math
import time


class BodyMotionPlanner():
    def __init__(self , cmd, leg, body, gait_planner):
        self.cmd = cmd
        self.body = body
        self.leg = leg
        self.gait = gait_planner

        self.prev_slant = self.cmd.body.slant

        # self.__L1 = self.leg.physical._L1
        # self.__L2 = self.leg.physical._L2
        # self.__L3 = self.leg.physical._L3

        self.cmd.leg.foot_zero_pnt[:,1] = 0.0

    def set_init_pose(self):
        self.body.roll = 0
        self.body.pitch = 0
        self.body.yaw = 0

        self.cmd.leg.foot_zero_pnt[:,2] = np.array(self.cmd.body.height)
        self.leg.RF.pose.cur_coord[:] = np.array(self.cmd.leg.foot_zero_pnt[0,:])
        self.leg.LF.pose.cur_coord[:] = np.array(self.cmd.leg.foot_zero_pnt[1,:])
        self.leg.LR.pose.cur_coord[:] = np.array(self.cmd.leg.foot_zero_pnt[2,:])
        self.leg.RR.pose.cur_coord[:] = np.array(self.cmd.leg.foot_zero_pnt[3,:])
        return True

    def change_height(self):
        self.leg.RF.pose.cur_coord[2] = self.body.height[:]
        self.leg.LF.pose.cur_coord[2] = self.body.height[:]
        self.leg.LR.pose.cur_coord[2] = self.body.height[:]
        self.leg.RR.pose.cur_coord[2] = self.body.height[:]
        return True
    
    def run(self):
        while True:
            self.cmd.leg.foot_zero_pnt[:,2] = np.array(self.cmd.body.height) 
            """ uncomment below 2 lines to activate slant from joystick"""
            self.cmd.leg.foot_zero_pnt[0,0] = 0.13 * np.cos(math.pi/4) #+ self.cmd.body.slant[0]
            self.cmd.leg.foot_zero_pnt[1,0] = -0.13* np.cos(math.pi/4) #+ self.cmd.body.slant[0]
            self.cmd.leg.foot_zero_pnt[2,0] = -0.13 * np.cos(math.pi/4) #+ self.cmd.body.slant[0]
            self.cmd.leg.foot_zero_pnt[3,0] = 0.13 * np.cos(math.pi/4) #+ self.cmd.body.slant[0]
            self.cmd.leg.foot_zero_pnt[:2,1] = 0.13 * np.sin(math.pi/4) #+ self.cmd.body.slant[1]
            self.cmd.leg.foot_zero_pnt[2:,1] = 0.13 * -np.sin(math.pi/4) #+ self.cmd.body.slant[1]
            self.body.roll = self.cmd.body.roll
            self.body.pitch = self.cmd.body.pitch
            self.body.yaw = self.cmd.body.yaw
            # if np.any(self.cmd.body.slant != self.prev_slant):
            #     self.leg.RF.pose.cur_coord[:2] = 
            self.leg.RF.pose.cur_coord[:] = np.array(self.cmd.leg.foot_zero_pnt[0,:]) + self.gait.RF_traj[:] + self.body.ZMP_handler[0,:]*np.array([1,1,0]) + self.cmd.body.slant[:]*np.array([1,1,0])
            self.leg.LF.pose.cur_coord[:] = np.array(self.cmd.leg.foot_zero_pnt[1,:]) + self.gait.LF_traj[:] + self.body.ZMP_handler[1,:]*np.array([1,1,0]) + self.cmd.body.slant[:]*np.array([1,1,0])
            self.leg.LR.pose.cur_coord[:] = np.array(self.cmd.leg.foot_zero_pnt[2,:]) + self.gait.LR_traj[:] + self.body.ZMP_handler[2,:]*np.array([1,1,0]) + self.cmd.body.slant[:]*np.array([1,1,0])
            self.leg.RR.pose.cur_coord[:] = np.array(self.cmd.leg.foot_zero_pnt[3,:]) + self.gait.RR_traj[:] + self.body.ZMP_handler[3,:]*np.array([1,1,0]) + self.cmd.body.slant[:]*np.array([1,1,0])
            
            time.sleep(0.0002)
        # print(self.leg.RF.pose.cur_coord[:]) 



# leg.RF.gait.displacement 
# body.roll