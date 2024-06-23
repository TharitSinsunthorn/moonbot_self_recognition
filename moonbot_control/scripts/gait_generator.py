#! /usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
# from time import time
import time
from mpl_toolkits.mplot3d import Axes3D
import math

from moonbot_variables import Body, Leg, Cmds
from IK.limb_kinematics import InvKinematics



class GaitPlanner():
    def __init__(self, cmd, leg, body):
        self.cmd = cmd
        self.leg = leg
        self.body = body

        self.gnd_touched = np.ones([4]) #rf, lf, lr, rr
        self.sample_time = 0.001 # default 0.001

        self.RF_traj = np.zeros([3])
        self.LF_traj = np.zeros([3])
        self.LR_traj = np.zeros([3])
        self.RR_traj = np.zeros([3])

        self.rf_traj = []
        self.lf_traj = []
        self.lr_traj = []
        self.rr_traj = []

        self.t_zmp_wavegait = 0.5
        self.len_zmp_wavegait = 0

        self.IK = InvKinematics()

    
    def swing_RF(self, t):
        traj_pnt = np.zeros([3])
        self.leg.RF.gait.swing.time =  self.cmd.gait.swing_time
        if self.cmd.mode.walk and np.any(self.cmd.gait.step_len != 0):
            self.leg.RF.gait.swing.end_pnt[:2] = np.array(self.cmd.gait.step_len)/2
            self.leg.RF.gait.swing.end_pnt[2] = self.leg.RF.pose.cur_coord[2]
            # set start point
            if not self.leg.RF.gait.swing.start:
                self.leg.RF.gait.stance.start = False
                self.leg.RF.gait.swing.start = True
                self.leg.RF.gait.swing.start_pnt[:2] = self.leg.RF.pose.cur_coord[:2] - self.cmd.leg.foot_zero_pnt[0,:2] - self.body.ZMP_handler[0,:2]
                self.leg.RF.gait.swing.start_pnt[2] = self.leg.RF.pose.cur_coord[2]
            
            # make trajectory
            T = self.leg.RF.gait.swing.time
            traj_pnt[:2] = self.leg.RF.gait.swing.start_pnt[:2] + (self.leg.RF.gait.swing.end_pnt[:2] - self.leg.RF.gait.swing.start_pnt[:2])*t/T 
            traj_pnt[2] = - np.array(self.cmd.gait.swing_step_h)* np.sin(t/T*np.pi)
            
            # end point
            if t >= T - self.sample_time*2:
                traj_pnt[:2] = np.array(self.leg.RF.gait.swing.end_pnt)[:2]
                traj_pnt[2] = 0
                self.leg.RF.gait.swing.start = False
                
        else:
            traj_pnt[:] = 0
            
        self.RF_traj = np.array(traj_pnt)
        

    def stance_RF(self, t):
        traj_pnt = np.zeros([3])
        self.leg.RF.gait.stance.time = self.cmd.gait.cycle_time - self.cmd.gait.swing_time
        if self.cmd.mode.walk and np.any(self.cmd.gait.step_len != 0):
            self.leg.RF.gait.stance.end_pnt[:2] = - np.array(self.cmd.gait.step_len)/2
            self.leg.RF.gait.stance.end_pnt[2] = self.leg.RF.pose.cur_coord[2]
            # set start point
            if not self.leg.RF.gait.stance.start:
                self.leg.RF.gait.stance.start = True
                self.leg.RF.gait.stance.start_pnt[:2] = self.leg.RF.pose.cur_coord[:2] - self.cmd.leg.foot_zero_pnt[0,:2] - self.body.ZMP_handler[0,:2]
                self.leg.RF.gait.stance.start_pnt[2] = self.leg.RF.pose.cur_coord[2]
            
            # make trajectory
            T = self.leg.RF.gait.stance.time
            traj_pnt[:2] = self.leg.RF.gait.stance.start_pnt[:2] + (self.leg.RF.gait.stance.end_pnt[:2] - self.leg.RF.gait.stance.start_pnt[:2])*t/T 
            traj_pnt[2] = np.array(self.cmd.gait.stance_step_h)* np.sin(t/T*np.pi)
            
            # end point
            if t >= T - self.sample_time*2:
                traj_pnt[:2] = self.leg.RF.gait.stance.end_pnt[:2]
                traj_pnt[2] = 0
                self.leg.RF.gait.stance.start = False
        else:
            traj_pnt[:] = 0

        self.RF_traj = np.array(traj_pnt) 
        

    def swing_LF(self, t):
        traj_pnt = np.zeros([3])
        self.leg.LF.gait.swing.time =  np.array(self.cmd.gait.swing_time)
        if self.cmd.mode.walk and np.any(self.cmd.gait.step_len != 0):
            self.leg.LF.gait.swing.end_pnt[:2] = np.array(self.cmd.gait.step_len)/2 
            self.leg.LF.gait.swing.end_pnt[2] = self.leg.LF.pose.cur_coord[2]
            # set start point
            if not self.leg.LF.gait.swing.start:
                self.leg.LF.gait.stance.start = False
                self.leg.LF.gait.swing.start = True
                self.leg.LF.gait.swing.start_pnt[:2] = self.leg.LF.pose.cur_coord[:2] - self.cmd.leg.foot_zero_pnt[1,:2] - self.body.ZMP_handler[1,:2]
                self.leg.LF.gait.swing.start_pnt[2] = self.leg.LF.pose.cur_coord[2]
            
            # make trajectory
            T = self.leg.LF.gait.swing.time
            traj_pnt[:2] = self.leg.LF.gait.swing.start_pnt[:2] + (self.leg.LF.gait.swing.end_pnt[:2] - self.leg.LF.gait.swing.start_pnt[:2])*t/T 
            traj_pnt[2] = - self.cmd.gait.swing_step_h* np.sin(t/T*np.pi)
            
            # end point
            if t >= T - self.sample_time*2:
                traj_pnt[:2] = np.array(self.leg.LF.gait.swing.end_pnt)[:2]
                traj_pnt[2] = 0
                self.leg.LF.gait.swing.start = False
                
        else:
            traj_pnt[:] = 0

        self.LF_traj = np.array(traj_pnt)


    def stance_LF(self, t):
        traj_pnt = np.zeros([3])
        self.leg.LF.gait.stance.time = self.cmd.gait.cycle_time - self.cmd.gait.swing_time
        if self.cmd.mode.walk and np.any(self.cmd.gait.step_len != 0):
            self.leg.LF.gait.stance.end_pnt[:2] = - self.cmd.gait.step_len/2
            self.leg.LF.gait.stance.end_pnt[2] = self.leg.LF.pose.cur_coord[2]
            # set start point
            if not self.leg.LF.gait.stance.start:
                self.leg.LF.gait.stance.start = True
                self.leg.LF.gait.stance.start_pnt[:2] = self.leg.LF.pose.cur_coord[:2] - self.cmd.leg.foot_zero_pnt[1,:2] - self.body.ZMP_handler[1,:2]
                self.leg.LF.gait.stance.start_pnt[2] = self.leg.LF.pose.cur_coord[2]

            # make trajectory
            T = self.leg.LF.gait.stance.time
            traj_pnt[:2] = self.leg.LF.gait.stance.start_pnt[:2] + (self.leg.LF.gait.stance.end_pnt[:2] - self.leg.LF.gait.stance.start_pnt[:2])*t/T 
            traj_pnt[2] = np.array(self.cmd.gait.stance_step_h)* np.sin(t/T*np.pi)
            
            # end point
            if t >= T - self.sample_time*2:
                traj_pnt[:2] = np.array(self.leg.LF.gait.stance.end_pnt)[:2]
                traj_pnt[2] = 0
                self.leg.LF.gait.stance.start = False
        else:
            traj_pnt[:] = 0
            
        self.LF_traj = np.array(traj_pnt)
        

    def swing_LR(self, t):
        traj_pnt = np.zeros([3])
        self.leg.LR.gait.swing.time =  self.cmd.gait.swing_time
        if self.cmd.mode.walk and np.any(self.cmd.gait.step_len != 0):
            self.leg.LR.gait.swing.end_pnt[:2] = np.array(self.cmd.gait.step_len)/2
            self.leg.LR.gait.swing.end_pnt[2] = self.leg.LR.pose.cur_coord[2]
            # set start point
            if not self.leg.LR.gait.swing.start:
                self.leg.LR.gait.stance.start = False
                self.leg.LR.gait.swing.start = True
                self.leg.LR.gait.swing.start_pnt[:2] = self.leg.LR.pose.cur_coord[:2] - self.cmd.leg.foot_zero_pnt[2,:2]  - self.body.ZMP_handler[2,:2]
                self.leg.LR.gait.swing.start_pnt[2] = self.leg.LR.pose.cur_coord[2]
            
            # make trajectory
            T = self.leg.LR.gait.swing.time
            traj_pnt[:2] = self.leg.LR.gait.swing.start_pnt[:2] + (self.leg.LR.gait.swing.end_pnt[:2] - self.leg.LR.gait.swing.start_pnt[:2])*t/T 
            traj_pnt[2] = - np.array(self.cmd.gait.swing_step_h)* np.sin(t/T*np.pi)
            
            # end point
            if t >= T - self.sample_time*2:
                traj_pnt[:2] = np.array(self.leg.LR.gait.swing.end_pnt)[:2]
                traj_pnt[2] = 0
                self.leg.LR.gait.swing.start = False
                
        else:
            traj_pnt[:] = 0

        self.LR_traj = np.array(traj_pnt)

    def stance_LR(self, t):
        traj_pnt = np.zeros([3])
        self.leg.LR.gait.stance.time = self.cmd.gait.cycle_time - self.cmd.gait.swing_time
        if self.cmd.mode.walk and np.any(self.cmd.gait.step_len != 0):
            self.leg.LR.gait.stance.end_pnt[:2] = - np.array(self.cmd.gait.step_len)/2
            self.leg.LR.gait.stance.end_pnt[2] = self.leg.LR.pose.cur_coord[2]
            # set start point
            if not self.leg.LR.gait.stance.start:
                self.leg.LR.gait.stance.start = True
                self.leg.LR.gait.stance.start_pnt[:2] = self.leg.LR.pose.cur_coord[:2] - self.cmd.leg.foot_zero_pnt[2,:2]  - self.body.ZMP_handler[2,:2]
                self.leg.LR.gait.stance.start_pnt[2] = self.leg.LR.pose.cur_coord[2]
            
            # make trajectory
            T = self.leg.LR.gait.stance.time
            traj_pnt[:2] = self.leg.LR.gait.stance.start_pnt[:2] + (self.leg.LR.gait.stance.end_pnt[:2] - self.leg.LR.gait.stance.start_pnt[:2])*t/T 
            traj_pnt[2] = np.array(self.cmd.gait.stance_step_h)* np.sin(t/T*np.pi)
            
            # end point
            if t >= T - self.sample_time*2:
                traj_pnt[:2] = np.array(self.leg.LR.gait.stance.end_pnt)[:2]
                traj_pnt[2] = 0
                self.leg.LR.gait.stance.start = False
        else:
            traj_pnt[:] = 0

        self.LR_traj = np.array(traj_pnt)
        

    def swing_RR(self, t):
        traj_pnt = np.zeros([3])
        self.leg.RR.gait.swing.time =  self.cmd.gait.swing_time
        if self.cmd.mode.walk and np.any(self.cmd.gait.step_len != 0):
            self.leg.RR.gait.swing.end_pnt[:2] = np.array(self.cmd.gait.step_len/2) #* np.array([1,-1])
            self.leg.RR.gait.swing.end_pnt[2] = self.leg.RR.pose.cur_coord[2]
            # set start point
            if not self.leg.RR.gait.swing.start:
                self.leg.RR.gait.stance.start = False
                self.leg.RR.gait.swing.start = True
                self.leg.RR.gait.swing.start_pnt[:2] = self.leg.RR.pose.cur_coord[:2] - self.cmd.leg.foot_zero_pnt[3,:2] - self.body.ZMP_handler[3,:2]
                self.leg.RR.gait.swing.start_pnt[2] = self.leg.RR.pose.cur_coord[2]
            
            # make trajectory
            T = self.leg.RR.gait.swing.time
            traj_pnt[:2] = self.leg.RR.gait.swing.start_pnt[:2] + (self.leg.RR.gait.swing.end_pnt[:2] - self.leg.RR.gait.swing.start_pnt[:2])*t/T 
            traj_pnt[2] = - self.cmd.gait.swing_step_h* np.sin(t/T*np.pi)
            
            # end point
            if t >= T - self.sample_time*2:
                traj_pnt[:2] = self.leg.RR.gait.swing.end_pnt[:2]
                traj_pnt[2] = 0
                self.leg.RR.gait.swing.start = False
                
        else:
            traj_pnt[:] = 0

        self.RR_traj = np.array(traj_pnt)

    def stance_RR(self, t):
        traj_pnt = np.zeros([3])
        self.leg.RR.gait.stance.time = self.cmd.gait.cycle_time - self.cmd.gait.swing_time
        if self.cmd.mode.walk and np.any(self.cmd.gait.step_len != 0):
            self.leg.RR.gait.stance.end_pnt[:2] = - self.cmd.gait.step_len/2 #* np.array([1,-1])
            self.leg.RR.gait.stance.end_pnt[2] = self.leg.RR.pose.cur_coord[2]
            # set start point
            if not self.leg.RR.gait.stance.start:
                self.leg.RR.gait.stance.start = True
                self.leg.RR.gait.stance.start_pnt[:2] = self.leg.RR.pose.cur_coord[:2] - self.cmd.leg.foot_zero_pnt[3,:2]  - self.body.ZMP_handler[3,:2]
                self.leg.RR.gait.stance.start_pnt[2] = self.leg.RR.pose.cur_coord[2]

            # make trajectory
            T = self.leg.RR.gait.stance.time
            traj_pnt[:2] = self.leg.RR.gait.stance.start_pnt[:2] + (self.leg.RR.gait.stance.end_pnt[:2] - self.leg.RR.gait.stance.start_pnt[:2])*t/T 
            traj_pnt[2] = self.cmd.gait.stance_step_h* np.sin(t/T*np.pi)
            
            # end point
            if t >= T - self.sample_time*2:
                traj_pnt[:2] = self.leg.RR.gait.stance.end_pnt[:2]
                traj_pnt[2] = 0
                self.leg.RR.gait.stance.start = False
        else:
            traj_pnt[:] = 0
            
        self.RR_traj = np.array(traj_pnt)
        

    def run_trot(self):
        t = time.time()
        dt = time.time() - t
        i = 0
        while self.cmd.mode.walk:
            if dt <= self.cmd.gait.cycle_time:
                if dt >= self.sample_time*i:
                    i += 1
                    # RF,LR - swing |   LF,RR - stance
                    if dt <= self.cmd.gait.swing_time:
                        self.swing_RF(dt)
                        self.stance_LF(dt)
                        self.stance_RR(dt)
                        self.swing_LR(dt)
                    # All - stance
                    elif dt > self.cmd.gait.swing_time and dt < self.cmd.gait.cycle_time - self.cmd.gait.swing_time:
                        self.stance_RF(dt - self.cmd.gait.swing_time)
                        self.stance_LF(dt)
                        self.stance_RR(dt)
                        self.stance_LR(dt - self.cmd.gait.swing_time)
                    # RF,LR - stance |   LF,RR - swing
                    else:
                        stance_t = self.cmd.gait.cycle_time - self.cmd.gait.swing_time
                        self.stance_RF(dt - self.cmd.gait.swing_time)
                        self.swing_LF(dt - stance_t)
                        self.swing_RR(dt - stance_t)
                        self.stance_LR(dt - self.cmd.gait.swing_time)
            else:
                # cycle reset
                i = 0
                t = time.time()
                
            # print(self.leg.RF.gait.traj_pnt[:]) #debug
            dt = time.time() - t
            time.sleep(0.0002)

    def run_trot2(self):
        t = time.time()
        dt = time.time() - t
        i = 0
        while self.cmd.mode.walk:
            if dt <= self.cmd.gait.cycle_time:
                if dt >= self.sample_time*i:
                    i += 1
                    # FR,BL - swing |   FL,BR - stance
                    if dt <= self.cmd.gait.swing_time:
                        self.swing_RF(dt)
                        self.stance_LF(dt)
                        self.stance_RR(dt)
                        self.swing_LR(dt)
                    # All - stance
                    elif dt > self.cmd.gait.swing_time and dt < (self.cmd.gait.cycle_time - self.cmd.gait.swing_time)/2:
                        self.stance_RF(dt - self.cmd.gait.swing_time)
                        self.stance_LF(dt)
                        self.stance_RR(dt)
                        self.stance_LR(dt - self.cmd.gait.swing_time)
                    # FR,BL - stance |   FL,BR - swing
                    else:
                        stance_t = self.cmd.gait.cycle_time - self.cmd.gait.swing_time
                        self.stance_RF(dt - self.cmd.gait.swing_time)
                        self.swing_LF(dt - stance_t)
                        self.swing_RR(dt - stance_t)
                        self.stance_LR(dt - self.cmd.gait.swing_time)
            else:
                # cycle reset
                i = 0
                t = time.time()
            # print(self.leg.RF.gait.traj_pnt[:]) #debug
            dt = time.time() - t
            time.sleep(0.0002)

    def run_CrawlGait(self):
        stance_zone_count = np.zeros([4])
        t = time.time()
        dt = time.time() - t
        t_zmp = 0.5
        zmp_len = 0.01
        i = 0
        self.body.ZMP_handler[:,:] = 0

        while self.cmd.mode.walk :
            zone_time = self.cmd.gait.cycle_time/4
            if dt <= self.cmd.gait.cycle_time + 2*t_zmp:
                if dt >= self.sample_time*i:
                    i += 1
                    # ZMP body move left
                    if dt <= t_zmp/2:
                        if np.any(self.cmd.gait.step_len[:2] != 0):
                            self.body.ZMP_handler[:,0] = zmp_len * dt/(t_zmp/2) 
                        else: 
                            self.body.ZMP_handler[:,:] = 0

                    # BR - swing |   other - stance
                    elif dt > t_zmp/2 and dt <= zone_time + t_zmp/2:
                        self.swing_RR(dt - t_zmp/2)
                        self.stance_RF(stance_zone_count[0]*zone_time + dt - t_zmp/2) 
                        self.stance_LR(stance_zone_count[3]*zone_time + dt - t_zmp/2)
                        self.stance_LF(dt - t_zmp/2)
                        stance_zone_count[2] = 0
                    # FR - swing | other stance
                    elif dt > zone_time + t_zmp/2 and dt <= 2*zone_time + t_zmp/2:
                        self.stance_RR(dt - (zone_time + t_zmp/2))
                        self.swing_RF(dt - (zone_time + t_zmp/2))
                        self.stance_LR(stance_zone_count[3]*zone_time + dt - t_zmp/2)
                        self.stance_LF(dt - t_zmp/2)
                        stance_zone_count[0] = 0

                    # ZMP move body right
                    elif dt > 2*zone_time + t_zmp/2 and dt <= 2*zone_time + 3*t_zmp/2:
                        if np.any(self.cmd.gait.step_len[:2] != 0):
                            self.body.ZMP_handler[:,0] = zmp_len-(dt-2*zone_time - t_zmp/2) * zmp_len/(t_zmp/2) 
                        else:
                            self.body.ZMP_handler[:,:] = 0

                    # BL - swing | other - stance
                    elif dt > 2*zone_time + 3*t_zmp/2 and dt <= 3*zone_time + 3*t_zmp/2:
                        self.stance_RR(dt - (zone_time + 3/2*t_zmp))
                        self.stance_RF(dt - (2*zone_time + 3/2*t_zmp))
                        self.swing_LR(dt - (2*zone_time + 3/2*t_zmp))
                        self.stance_LF(dt - 3/2*t_zmp)
                        stance_zone_count[3] = 0
                        
                    # FL - swing | other - stance
                    elif dt > 3*zone_time + 3/2*t_zmp and dt <= 4*zone_time + 3/2*t_zmp:
                        self.stance_RR(dt - (zone_time + 3/2*t_zmp))
                        self.stance_RF(dt - (2*zone_time + 3/2*t_zmp))
                        self.stance_LR(dt - (3*zone_time + 3/2*t_zmp))
                        self.swing_LF(dt - (3*zone_time + 3/2*t_zmp))
                        stance_zone_count[1] = 0

                    else:
                        if np.any(self.cmd.gait.step_len[:2] != 0):
                            self.body.ZMP_handler[:,0] = -zmp_len + (dt-4*zone_time - 3/2*t_zmp)*zmp_len/(t_zmp/2)
                        else:
                            self.body.ZMP_handler[:,:] = 0  
            else:
                if np.any(self.cmd.gait.step_len[:2] != 0):
                    stance_zone_count[0] = 2    # FR
                    stance_zone_count[1] = 0    # FL
                    stance_zone_count[2] = 3    # BR
                    stance_zone_count[3] = 1    # BL
                else:
                    for i in range (4):
                        stance_zone_count[i] = 0
                # cycle reset
                self.body.ZMP_handler[:,:] = 0
                i = 0
                t = time.time()

            dt = time.time() - t
            time.sleep(0.0001)
    
    def give_hand(self):
        self.body.ZMP_handler[:,0] = self.len_zmp_wavegait 
           
        while self.cmd.mode.gait_type == 0:
            self.RF_traj[0] = self.cmd.gait.step_len[0]
            self.RF_traj[2] = -self.cmd.gait.step_len[1]
           

    def run(self):
        while True:
            if self.cmd.mode.walk and np.any(self.cmd.gait.step_len[:] != 0):
                if self.cmd.mode.gait_type == 1:
                    self.cmd.gait.cycle_time = 1.0
                    self.cmd.gait.swing_time = 0.5* self.cmd.gait.cycle_time
                    self.body.ZMP_handler[:,:] = 0  
                    self.run_trot()
                elif self.cmd.mode.gait_type == 2:
                    self.cmd.gait.cycle_time = 3.0
                    self.cmd.gait.swing_time = 0.25 * self.cmd.gait.cycle_time
                    self.body.ZMP_handler[:,:] = 0
                    self.run_CrawlGait()
                elif self.cmd.mode.gait_type == 3:
                    self.cmd.gait.cycle_time = 0.8
                    self.cmd.gait.swing_time = 0.25 * self.cmd.gait.cycle_time
                    self.body.ZMP_handler[:,:] = 0
                    self.run_trot2()
                elif self.cmd.mode.gait_type == 0:
                    self.give_hand()
                    # pass
                
            else:
                self.RF_traj[2] = 0
                self.LF_traj[2] = 0
                self.LR_traj[2] = 0
                self.RR_traj[2] = 0
                self.body.ZMP_handler[:,:] = 0
        

    
# if __name__ == '__main__':
#     GaitPlanner(Cmds(), Leg(), Body()).run()