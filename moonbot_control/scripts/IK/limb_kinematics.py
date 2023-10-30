from IK import parameters as params
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

        self.MAX_coord_dist = self.L1 + self.L2 + self.L3
        self.MIN_coord_dist = np.sqrt((self.L1-self.L3)**2 + self.L2**2)

        self.offset = [0.24, 0.24]

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
        r = np.linalg.norm(point)

        if r >= self.MIN_coord_dist and r <= self.MAX_coord_dist:
            return False
        else:
            return True


    def get_joint_angles(self, coord, rot = [0, 0, 0]):
        
        x, y, z = coord
        th = [0,0,0]
        
        L1 = self.L1
        L2 = self.L2
        L3 = self.L3

        LL = np.hypot(x, y)
        D = np.sqrt(z**2 + (LL-L1)**2)

        a = (L3**2 - L2**2 - D**2)/(-2*D*L2)
        b = (D**2 - L2**2 - L3**2)/(-2*L2*L3)

        j1 = np.arctan(y/x)
        if not np.isnan(j1) and j1 <= 1.57 and j1 >= -1.57:
            th[0] = j1
        elif j1 > 1.57:
            th[0] = 1.57
        elif j1 < -1.57:
            th[0] = -1.57

        if a > 1 or a < -1 or b > 1 or b < -1:
            pass
        else:
            j2 = -np.arctan(z/(LL-L1)) + np.arccos((L3**2 - L2**2 - D**2)/(-2*D*L2))
            if not np.isnan(j2) and j2 <= 0.756 and j2 >= -1.57:
                th[1] = j2
            elif j2 > 0.756:
                th[1] = 0.756
            elif j3 < -1.57:
                th[1] = -1.57

            j3 = np.arccos((D**2 - L2**2 - L3**2)/(-2*L2*L3)) - math.pi
            if not np.isnan(j3) and j3 <= 1.57 and j3 >= -1.57:
                th[2] = j3
            elif j3 > 1.57:
                th[2] = 1.57
            elif j3 < -1.57:
                th[2] = -1.57

        return th @ self.rotMat(rot)


    # def get_FR_joint_angles(self, coord, eularAng):
    #     # rotate around midle of the body
    #     translate_FR = self.offset*self.M_F*self.M_R
    #     coord_ = (np.dot((coord * self.M_R + translate_FR), self.rotMat(eularAng)) - translate_FR) * self.M_R
    #     # check singularity of the legs
    #     if self.is_singularity(coord_):
    #             self.singularity[0] = True
    #     else:
    #         self.singularity[0] = False
    #     #  check if any legs is in singularity
    #     if any(self.singularity):
    #         self.legState.FR.now_angles = self.legState.FR.prev_angles
    #     # if no singularities, get new joint angles
    #     else:
    #         self.legState.FR.now_angles = self.get_joint_angles(coord_)
    #         self.legState.FR.prev_angles = self.legState.FR.now_angles
    #         self.singularity[0] = False
    #     return self.legState.FR.now_angles

    # def get_FL_joint_angles(self, coord, eularAng):
    #     translate_FL = self.offset*self.M_F*self.M_L
    #     coord_ = (np.dot((coord * self.M_L + translate_FL), self.rotMat(eularAng)) - translate_FL) * self.M_L
    #     # check singularity of the legs
    #     if self.is_singularity(coord_):
    #             self.singularity[1] = True
    #     else:
    #         self.singularity[1] = False
    #     if any(self.singularity):
    #         self.legState.FL.now_angles = self.legState.FL.prev_angles
    #     # if no singularities, get new joint angles
    #     else:
    #         self.legState.FL.now_angles = self.get_joint_angles(coord_)
    #         self.legState.FL.prev_angles = self.legState.FL.now_angles
    #         self.singularity[1] = False
    #     return self.legState.FL.now_angles
     

    # def get_BR_joint_angles(self, coord, eularAng):
    #     translate_BR = self.offset*self.M_B*self.M_R
    #     coord_ = (np.dot((coord * self.M_R + translate_BR), self.rotMat(eularAng)) - translate_BR) * self.M_R
    #     # check singularity of the legs
    #     if self.is_singularity(coord_):
    #             self.singularity[2] = True
    #     else:
    #         self.singularity[2] = False
    #     if any(self.singularity):
    #         self.legState.BR.now_angles = self.legState.BR.prev_angles
    #     # if no singularities, get new joint angles
    #     else:
    #         self.legState.BR.now_angles = self.get_joint_angles(coord_)
    #         self.legState.BR.prev_angles = self.legState.BR.now_angles
    #         self.singularity[2] = False
    #     return self.legState.BR.now_angles

    # def get_BL_joint_angles(self, coord, eularAng):
    #     translate_FL = self.offset*self.M_B*self.M_L
    #     coord_ = (np.dot((coord * self.M_L + translate_FL), self.rotMat(eularAng)) - translate_FL) * self.M_L
    #     # check singularity of the legs
    #     if self.is_singularity(coord_):
    #             self.singularity[3] = True
    #     else:
    #         self.singularity[3] = False
    #     if any(self.singularity):
    #         self.legState.BL.now_angles = self.legState.BL.prev_angles
    #     # if no singularities, get new joint angles
    #     else:
    #         self.legState.BL.now_angles = self.get_joint_angles(coord_)
    #         self.legState.BL.prev_angles = self.legState.BL.now_angles
    #         self.singularity[3] = False
    #     return self.legState.BL.now_angles



