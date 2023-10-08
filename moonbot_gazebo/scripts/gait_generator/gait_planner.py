import numpy as np
from time import time
import math 

class GaitPlanner():
	def __init__(self, cmd, leg, body):
		self.cmd = cmd
		self.leg = leg 
		self.body = body 

		self.gnd_touched = np.ones([4]) #fr,fl,br,bl
		self.sample_time = 0.001

		self.FR_traj = np.zeros([3])
        self.FL_traj = np.zeros([3])
        self.BR_traj = np.zeros([3])
        self.BL_traj = np.zeros([3])

        self.fr_traj = []
        self.fl_traj = []
        self.br_traj = []
        self.bl_traj = []


    




	def run(self):
		while True:
