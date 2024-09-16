#! /usr/bin/env python3

import numpy as np
import time

class GaitPlanner():
    def __init__(self, cmd, leg, body):
        self.cmd = cmd
        self.leg = leg
        self.body = body

        self.gnd_touched = np.ones([4]) #rf, lf, lr, rr
        self.sample_time = 0.001 # default 0.001

        self.t_zmp_crawlgait = 0.5
        self.len_zmp_wavegait = 0

        self.leg_names = ['RF', 'LF', 'LR', 'RR']
        self.traj = {'RF': np.zeros([3]), 'LF': np.zeros([3]), 'LR': np.zeros([3]), 'RR': np.zeros([3])}
        

    def swing_leg(self, leg_name, t, end):
        traj_pnt = np.zeros([3])
        leg = getattr(self.leg, leg_name)
        leg.gait.swing.time = self.cmd.gait.swing_time
        if self.cmd.mode.walk and np.any(self.cmd.gait.step_len != 0):
            leg.gait.swing.end_pnt[:2] = end
            leg.gait.swing.end_pnt[2] = leg.pose.cur_coord[2]
            # Set start point
            if not leg.gait.swing.start:
                leg.gait.stance.start = False
                leg.gait.swing.start = True
                leg.gait.swing.start_pnt[:2] = leg.pose.cur_coord[:2] - self.cmd.leg.foot_zero_pnt[self.leg_names.index(leg_name), :2] - self.body.ZMP_handler[self.leg_names.index(leg_name), :2]
                leg.gait.swing.start_pnt[2] = leg.pose.cur_coord[2]
                # leg.gait.swing.start_pnt[:] = np.zeros(3)

            # Make trajectory
            T = leg.gait.swing.time
            traj_pnt[:2] = leg.gait.swing.start_pnt[:2] + (leg.gait.swing.end_pnt[:2] - leg.gait.swing.start_pnt[:2]) * t / T
            traj_pnt[2] = -self.cmd.gait.swing_step_h * np.sin(t / T * np.pi)

            # End point
            if t >= T - self.sample_time * 2:
                traj_pnt[:2] = np.array(leg.gait.swing.end_pnt)[:2]
                traj_pnt[2] = 0
                leg.gait.swing.start = False
        else:
            traj_pnt[:] = 0

        self.traj[leg_name] = traj_pnt

    def stance_leg(self, leg_name, t, end):
        traj_pnt = np.zeros([3])
        leg = getattr(self.leg, leg_name)
        leg.gait.stance.time = self.cmd.gait.cycle_time - self.cmd.gait.swing_time
        if self.cmd.mode.walk and np.any(self.cmd.gait.step_len != 0):
            leg.gait.stance.end_pnt[:2] = end
            leg.gait.stance.end_pnt[2] = leg.pose.cur_coord[2]
            if not leg.gait.stance.start:
                leg.gait.swing.start = False
                leg.gait.stance.start = True
                leg.gait.stance.start_pnt[:2] = leg.pose.cur_coord[:2] - self.cmd.leg.foot_zero_pnt[self.leg_names.index(leg_name), :2] - self.body.ZMP_handler[self.leg_names.index(leg_name), :2]
                leg.gait.stance.start_pnt[2] = leg.pose.cur_coord[2]
                # leg.gait.stance.start_pnt[:] = 0

            T = leg.gait.stance.time
            traj_pnt[:2] = leg.gait.stance.start_pnt[:2] + (leg.gait.stance.end_pnt[:2] - leg.gait.stance.start_pnt[:2]) * t / T
            traj_pnt[2] = self.cmd.gait.swing_step_h * np.sin(t / T * np.pi) / 4

            if t >= T - self.sample_time * 2:
                traj_pnt[:2] = np.array(leg.gait.stance.end_pnt)[:2]
                traj_pnt[2] = 0
                leg.gait.stance.start = False
        else:
            traj_pnt[:] = 0

        self.traj[leg_name] = traj_pnt

    def swing_RF(self, t):
        self.swing_leg('RF', t, self.cmd.gait.step_len / 2)

    def stance_RF(self, t):
        self.stance_leg('RF', t, -self.cmd.gait.step_len / 2)

    def swing_LF(self, t):
        self.swing_leg('LF', t, self.cmd.gait.step_len / 2)

    def stance_LF(self, t):
        self.stance_leg('LF', t, -self.cmd.gait.step_len / 2)

    def swing_LR(self, t):
        self.swing_leg('LR', t, self.cmd.gait.step_len / 2)

    def stance_LR(self, t):
        self.stance_leg('LR', t, -self.cmd.gait.step_len / 2)

    def swing_RR(self, t):
        self.swing_leg('RR', t, self.cmd.gait.step_len / 2)

    def stance_RR(self, t):
        self.stance_leg('RR', t, -self.cmd.gait.step_len / 2)

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
            dt = time.time() - t
            time.sleep(0.0002)

    def run_CrawlGait(self):
        stance_zone_count = np.zeros([4])
        t = time.time()
        dt = time.time() - t
        t_zmp = self.t_zmp_crawlgait
        zmp_len = 0.015
        i = 0
        self.body.ZMP_handler[:,:] = 0

        while self.cmd.mode.walk and np.any(self.cmd.gait.step_len[:2] != 0):
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

                    # RR - swing |   other - stance
                    elif dt > t_zmp/2 and dt <= zone_time + t_zmp/2:
                        self.swing_RR(dt - t_zmp/2)
                        self.stance_RF(stance_zone_count[0]*zone_time + dt - t_zmp/2) 
                        self.stance_LR(stance_zone_count[3]*zone_time + dt - t_zmp/2)
                        self.stance_LF(dt - t_zmp/2)
                        # stance_zone_count[2] = 0
                    # RF - swing | other stance
                    elif dt > zone_time + t_zmp/2 and dt <= 2*zone_time + t_zmp/2:
                        self.stance_RR(dt - (zone_time + t_zmp/2))
                        self.swing_RF(dt - (zone_time + t_zmp/2))
                        self.stance_LR(stance_zone_count[3]*zone_time + dt - t_zmp/2)
                        self.stance_LF(dt - t_zmp/2)
                        # stance_zone_count[0] = 0

                    # ZMP move body right
                    elif dt > 2*zone_time + t_zmp/2 and dt <= 2*zone_time + 3*t_zmp/2:
                        if np.any(self.cmd.gait.step_len[:2] != 0):
                            self.body.ZMP_handler[:,0] = zmp_len-(dt-2*zone_time - t_zmp/2) * zmp_len/(t_zmp/2) 
                        else:
                            self.body.ZMP_handler[:,:] = 0

                    # LR - swing | other - stance
                    elif dt > 2*zone_time + 3*t_zmp/2 and dt <= 3*zone_time + 3*t_zmp/2:
                        self.stance_RR(dt - (zone_time + 3/2*t_zmp))
                        self.stance_RF(dt - (2*zone_time + 3/2*t_zmp))
                        self.swing_LR(dt - (2*zone_time + 3/2*t_zmp))
                        self.stance_LF(dt - 3/2*t_zmp)
                        # stance_zone_count[3] = 0
                        
                    # LF - swing | other - stance
                    elif dt > 3*zone_time + 3/2*t_zmp and dt <= 4*zone_time + 3/2*t_zmp:
                        self.stance_RR(dt - (zone_time + 3/2*t_zmp))
                        self.stance_RF(dt - (2*zone_time + 3/2*t_zmp))
                        self.stance_LR(dt - (3*zone_time + 3/2*t_zmp))
                        self.swing_LF(dt - (3*zone_time + 3/2*t_zmp))
                        # stance_zone_count[1] = 0

                    else:
                        if np.any(self.cmd.gait.step_len[:2] != 0):
                            self.body.ZMP_handler[:,0] = -zmp_len + (dt-4*zone_time - 3/2*t_zmp)*zmp_len/(t_zmp/2)
                        else:
                            self.body.ZMP_handler[:,:] = 0  
            else:
                if np.any(self.cmd.gait.step_len[:2] != 0):
                    stance_zone_count[0] = 2    # RF
                    stance_zone_count[1] = 0    # LF
                    stance_zone_count[2] = 3    # LR
                    stance_zone_count[3] = 1    # RR
                else:
                    stance_zone_count[:] = 0
                    
                # cycle reset
                self.body.ZMP_handler[:,:] = 0
                i = 0
                t = time.time()

            dt = time.time() - t
            time.sleep(0.0001)
            
            
        self.traj = {leg: np.zeros(3) for leg in self.leg_names}
        self.body.ZMP_handler[:,:] = 0
                
        for i, leg_name in enumerate(self.leg_names):
            leg = getattr(self.leg, leg_name)
            leg.gait.stance.start = False
            leg.gait.swing.start = False
            
    def give_hand(self):
        self.body.ZMP_handler[:,0] = self.len_zmp_wavegait 
           
        while self.cmd.mode.gait_type == 0:
            self.traj['LF'][0] = self.cmd.gait.step_len[0]
            self.traj['LF'][1] = self.cmd.gait.step_len[1]
           

    def run(self):
        while True:
            if self.cmd.mode.walk:
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
                    self.cmd.gait.cycle_time = 3.0
                    self.cmd.gait.swing_time = 0.25 * self.cmd.gait.cycle_time
                    self.body.ZMP_handler[:,:] = 0
                    self.run_trot2()
                elif self.cmd.mode.gait_type == 0:
                    self.give_hand()
                
            else:
                # self.traj = {leg: np.zeros(3) for leg in self.leg_names}
                # self.body.ZMP_handler[:,:] = 0
                
                # for i, leg_name in enumerate(self.leg_names):
                #     leg = getattr(self.leg, leg_name)
                #     leg.gait.stance.start = False
                #     leg.gait.swing.start = False
                pass
                    
                    
# if __name__ == '__main__':
#     GaitPlanner(Cmds(), Leg(), Body()).run()