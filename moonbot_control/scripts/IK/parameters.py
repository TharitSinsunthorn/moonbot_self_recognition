import numpy as np
import math


'''
Joint nomeclature:
    joint1: Coxa
    joint2: Femur
    joint3: Tibia
'''

L1 = 0.065 # Length between joint1 (Near the base joint) and joint2
L2 = 0.125 # Length between joint2 and joint3 (Near the Tip Joint)
L3 = 0.160 # Length between Joint3 and Tip

D1 = 0.240 # Distance between Origin of base and origin of the joint1

'''
limp_pos contains the angle of the orientation of the limb w.r.t. base frame
'''
limb_angle = [45, 135, -135, -45]


offset_x = D1
offset_y = D1
coxa_length = L1 
femur_length = L2 
tibia_length = L3

coxa_upperlimit = math.pi/2
coxa_lowerlimit = -math.pi/2

femur_upperlimit = math.pi/4
femur_lowerlimit = -math.pi/2

tibia_upperlimit = math.pi/2
tibia_lowerlimit = -math.pi/2


# Motion related
sec = 1.0
span = 0.13
foward = -0.07
height = 0.24
