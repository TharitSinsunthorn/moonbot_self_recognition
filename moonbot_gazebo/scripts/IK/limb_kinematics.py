from IK import params as params
import numpy as np
import math

'''
Joint nomeclature:
    joint1: joint near the base
    joint2: joint in-between
    joint3: joint near the tip
'''

L1 = params.L1
L2 = params.L2
L3 = params.L3


class LegStates():
    prev_coord = None
    now_coord = None
    prev_angles = None
    now_angles = None
    in_singularity = None

class Legs():
    port_1 = LegStates()
    port_2 = LegStates()
    port_3 = LegStates()
    port_4 = LegStates()
    

# def bound_range(angle):
#     angle = angle%360
#     angle = (angle + 360) % 360
#     if (angle > 180):
#         angle-=360
#     return angle

class InvKinematics():
    def __init__(self):
        self.L1 = L1
        self.L2 = L2
        self.L3 = L3

        # self.offset = [0,0]

        self.M_R = np.array([1, 1, 1])
        self.M_L = np.array([1, -1, 1])
        self.M_F = np.array([1, 1, 1])
        self.M_B = np.array([-1, 1, 1])

        self.legsState = Legs()

        self.in_singularity = [0,0,0,0]


    def rotMat(self, eularAng):
        """
            eularAng: np array [roll, pitch, yaw]
            return : rotation matrix
        """
        M11 = np.cos(eularAng[1])*np.cos(eularAng[2])
        M12 = np.sin(eularAng[0])*np.sin(eularAng[1])*np.cos(eularAng[2]) - np.cos(eularAng[0])*np.sin(eularAng[2])
        M13 = np.cos(eularAng[0])*np.sin(eularAng[1])*np.cos(eularAng[2]) + np.sin(eularAng[0])*np.sin(eularAng[2])

        M21 = np.cos(eularAng[1])*np.sin(eularAng[2])
        M22 = np.sin(eularAng[0])*np.sin(eularAng[1])*np.sin(eularAng[2]) + np.cos(eularAng[0])*np.cos(eularAng[2])
        M23 = np.cos(eularAng[0])*np.sin(eularAng[1])*np.sin(eularAng[2]) - np.sin(eularAng[0])*np.cos(eularAng[2])
        
        M31 = -np.sin(eularAng[1])
        M32 = np.sin(eularAng[0])*np.cos(eularAng[1])
        M33 = np.cos(eularAng[0])*np.cos(eularAng[1])
        rotMat = np.array([ [M11, M12, M13], [M21, M22, M23], [M31, M32, M33] ])
        return rotMat


    def is_singularity(self, coord):
        coord = np.array(coord)

        # Calculate the distance from the origin coxa joint
        dist = np.linalg.norm(point)


    def get_joint_angles(self, coord):
        
        x, y, z = coord
        
        L1 = self.L1
        L2 = self.L2
        L3 = self.L3

        LL = np.hypot(x, y)
        D = np.sqrt(z**2 + (LL-L1)**2)
        
        th1 = np.arctan(y/x)
        th2 = np.arctan(z/(LL-L1)) - np.arccos((L3**2 - L2**2 - D**2)/(-2*D*L2))
        th3 = np.arccos((D**2 - L2**2 - L3**2)/(-2*L2*L3)) - math.pi

        return [th1, th2, th3] # The negative sign in third angle is because third servo is acting in opposite direction










# def forward_kinematics(th):
#     ''' Returns the Forward Kinematics of the Limb

#     Args:
#         joint_angles: an array containing the joint angle value (in deg) at each joint
#         leg_num: leg number between 1 to 4
#     Returns:
#         an array containing the x,y,z position (in m) of the limp tip in frame of reference of Base
#     '''
#     L1 = params.L1
#     L2 = params.L2
#     L3 = params.L3
#     Th1, Th2, Th3 = th

#     Th1 = math.radians(Th1)
#     Th2 = math.radians(Th2)
#     Th3 = -math.radians(Th3) # The negative sign in third angle is because third servo is acting in opposite direction

#     ### Forward Kinematics

#     x = math.cos(Th1)* (L1 + L2*math.cos(Th2) + L3*math.cos(Th2+Th3))
#     y = math.sin(Th1)* (L1 + L2*math.cos(Th2) + L3*math.cos(Th2+Th3))
#     z = L2*math.sin(Th2) + L3*math.sin(Th2+Th3)

#     '''
#     Coordinate Transformation: Joint 1 to Base Frame
#     '''
#     theta = math.radians(params.limb_angle[leg_num-1])

#     ### Rotation
    
#     xb = x * math.cos(theta) - y * math.sin(theta)
#     yb = x * math.sin(theta) + y * math.cos(theta)
#     zb = z

#     #### Translation ####
#     xb = xb + params.D1 * math.cos(theta)
#     yb = yb + params.D1 * math.sin(theta)

#     return [xb, yb, zb]





