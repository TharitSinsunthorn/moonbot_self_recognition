#! /usr/bin/env python3

import numpy as np
from IK import parameters as params


####### PRIVATE #######
class Cmds():
    class _mode():
        start = False
        walk = False
        side_walk_mode = 0
        gait_type = 0
    # -----------------
    class _body():
        height = 0.14
        roll = 0
        pitch = 0
        yaw = 0
        slant = np.zeros([3])
   
    # -----------------
    class _leg():
        foot_zero_pnt = np.zeros([4,3]) # [FR,FL,BR,BL][x,y,z]
    # ------------------
    class _gait():
        step_len = np.zeros([2]) # [len_x,len_y]
        swing_step_h = 0
        stance_step_h = 0
        cycle_time = 0
        swing_time = 0

    mode = _mode()
    body = _body()
    leg = _leg()
    gait = _gait()


#============================================================================ 
        
class Body():

    def __init__(self):
        self.height = None
        self.centerOfMass = np.zeros([3])
        self.physical = self._physiacal_params()
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.ZMP_handler = np.zeros([4,3]) #Zero Moment Point

    class _physiacal_params():
        _length = 2*params.offset_y
        _width = 2*params.offset_x
        _min_height = 0.14
        _max_height = 0.24
     


#============================================================================

class _LegParams:

    def __init__(self):
        self.pose = self.leg_pose()
        self.gait = self.gait_params() 

    class gait_params:
        def __init__(self):
            self.cycle_time = None
            self.step_len = np.zeros([3])
            self.stance = self.params()
            self.swing = self.params()
            self.traj_pnt = np.zeros([3]) # [x,y,z]

        class params:
            def __init__(self):
                self.start = False
                self.time = None
                self.start_pnt = np.zeros([3])
                self.end_pnt = np.zeros([3])


    class leg_pose:
        def __init__(self):
            self.zero_pnt = np.zeros([3])
            self.cur_coord = np.zeros([3])
            self.origin_2_endEfector_dist = np.sqrt(self.cur_coord[0]**2 + self.cur_coord[1]**2 + self.cur_coord[2]**2)


####### PUBLIC #######
class Leg:

    def __init__(self):
        self.RF = _LegParams()
        self.LF = _LegParams()
        self.LR = _LegParams()
        self.RR = _LegParams()
        self.physical = self._physical_params()

    class _physical_params():
        _L1 = params.L1 # m
        _L2 = params.L2 # m
        _L3 = params.L3 # m