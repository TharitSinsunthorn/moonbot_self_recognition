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
offset_x = params.offset_x
offset_y = params.offset_y
offset = [offset_x, offset_y, 0]

coxa_upperlimit = params.coxa_upperlimit
coxa_lowerlimit = params.coxa_lowerlimit

femur_upperlimit = params.femur_upperlimit
femur_lowerlimit = params.femur_lowerlimit

tibia_upperlimit = params.tibia_upperlimit
tibia_lowerlimit = params.tibia_lowerlimit


class LegStates():
    prev_coord = None
    now_coord = None
    prev_angles = None
    now_angles = None
    in_singularity = None

class Legs():
    RF = LegStates()
    LF = LegStates()
    LR = LegStates()
    RR = LegStates()
    

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

        self.offset = offset

        self.M_R = np.array([1, 1, 1])
        self.M_L = np.array([-1, 1, 1])
        self.M_F = np.array([1, 1, 1])
        self.M_Rr = np.array([1, -1, 1])

        self.legState = Legs()

        self.singularity = [0,0,0,0]


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
        r = np.linalg.norm(coord)

        if r >= self.MIN_coord_dist and r <= self.MAX_coord_dist:
            return False
        else:
            return True


    def get_joint_angles(self, coord, rot = [0, 0, 0]):
        
        coord = coord @ self.rotMat(rot)
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
        if not np.isnan(j1) and j1 <= coxa_upperlimit and j1 >= coxa_lowerlimit:
            th[0] = j1
        elif j1 > coxa_upperlimit:
            th[0] = coxa_upperlimit
        elif j1 < coxa_lowerlimit:
            th[0] = coxa_lowerlimit

        if a > 1 or a < -1 or b > 1 or b < -1:
            pass
        else:
            j2 = -np.arctan(z/(LL-L1)) + np.arccos((L3**2 - L2**2 - D**2)/(-2*D*L2))
            if not np.isnan(j2) and j2 <= femur_upperlimit and j2 >= femur_lowerlimit:
                th[1] = j2
            elif j2 > femur_upperlimit:
                th[1] = femur_upperlimit
            elif j3 < femur_lowerlimit:
                th[1] = femur_lowerlimit

            j3 = np.arccos((D**2 - L2**2 - L3**2)/(-2*L2*L3)) - math.pi
            if not np.isnan(j3) and j3 <= tibia_upperlimit and j3 >= tibia_lowerlimit:
                th[2] = j3
            elif j3 > tibia_upperlimit:
                th[2] = tibia_upperlimit
            elif j3 < tibia_lowerlimit:
                th[2] = tibia_lowerlimit

        return th


    def get_EE_position(self, th):
        L1, L2, L3 = L
        Th1, Th2, Th3 = th

        x0 = 0.0
        y0 = 0.0
        z0 = 0.0

        L1 = self.L1
        L2 = self.L2
        L3 = self.L3

        ##################################################
        # position of tip of link 1
        # x1 = L1*math.cos(Th2)*math.cos(Th1)
        # y1 = L1*math.cos(Th2)*math.sin(Th1)
        # z1 = L1*math.sin(Th2)

        x1 = L1*math.cos(Th1)
        y1 = L1*math.sin(Th1)
        z1 = 0.0

        # position of tip of link 2
        # x2 = x1 + L2*math.cos(Th2+Th3)*math.cos(Th1)
        # y2 = y1 + L2*math.cos(Th2+Th3)*math.sin(Th1)
        # z2 = z1 + L2*math.sin(Th2+Th3)
        ##################################################
        x2 = x1 + L2*math.cos(Th2)*math.cos(Th1)
        y2 = y1 + L2*math.cos(Th2)*math.sin(Th1)
        z2 = z1 + L2*math.sin(Th2)

        # position of tip of link 3
        x3 = x2 + L3*math.cos(Th2+Th3)*math.cos(Th1)
        y3 = y2 + L3*math.cos(Th2+Th3)*math.sin(Th1)
        z3 = z2 + L3*math.sin(Th2+Th3)
        print("pos: ", [x3,y3,z3])
        ##################################################

        X = np.array([[x3, y3, z3]])

        return X


    def get_RF_joint_angles(self, coord, eularAng):
        # rotate around midle of the body
        translate_RF = self.offset*self.M_F*self.M_R
        coord_ = np.dot((np.dot((coord + translate_RF), self.rotMat(eularAng)) - translate_RF), self.rotMat([0,0,1*math.pi/4]))
        # check singularity of the legs
        if self.is_singularity(coord_):
                self.singularity[0] = True
        else:
            self.singularity[0] = False
        #  check if any legs is in singularity
        if any(self.singularity):
            self.legState.RF.now_angles = self.legState.RF.prev_angles
        # if no singularities, get new joint angles
        else:
            self.legState.RF.now_angles = self.get_joint_angles(coord_)
            self.legState.RF.prev_angles = self.legState.RF.now_angles
            self.singularity[0] = False
        return self.legState.RF.now_angles

    def get_LF_joint_angles(self, coord, eularAng):
        translate_LF = self.offset*self.M_F*self.M_L
        coord_ = np.dot((np.dot((coord + translate_LF), self.rotMat(eularAng)) - translate_LF), self.rotMat([0,0,3*math.pi/4]))
        # check singularity of the legs
        if self.is_singularity(coord_):
                self.singularity[1] = True
        else:
            self.singularity[1] = False
        if any(self.singularity):
            self.legState.LF.now_angles = self.legState.LF.prev_angles
        # if no singularities, get new joint angles
        else:
            self.legState.LF.now_angles = self.get_joint_angles(coord_)
            self.legState.LF.prev_angles = self.legState.LF.now_angles
            self.singularity[1] = False
        return self.legState.LF.now_angles

    def get_LR_joint_angles(self, coord, eularAng):
        translate_LR = self.offset*self.M_Rr*self.M_L
        coord_ = np.dot((np.dot((coord + translate_LR), self.rotMat(eularAng)) - translate_LR), self.rotMat([0,0,5*math.pi/4]))
        # check singularity of the legs
        if self.is_singularity(coord_):
                self.singularity[3] = True
        else:
            self.singularity[3] = False
        if any(self.singularity):
            self.legState.LR.now_angles = self.legState.LR.prev_angles
        # if no singularities, get new joint angles
        else:
            self.legState.LR.now_angles = self.get_joint_angles(coord_)
            self.legState.LR.prev_angles = self.legState.LR.now_angles
            self.singularity[3] = False
        return self.legState.LR.now_angles


    def get_RR_joint_angles(self, coord, eularAng):
        translate_RR = self.offset*self.M_Rr*self.M_R
        coord_ = np.dot((np.dot((coord + translate_RR), self.rotMat(eularAng)) - translate_RR), self.rotMat([0,0,7*math.pi/4]))
        # check singularity of the legs
        if self.is_singularity(coord_):
                self.singularity[2] = True
        else:
            self.singularity[2] = False
        if any(self.singularity):
            self.legState.RR.now_angles = self.legState.RR.prev_angles
        # if no singularities, get new joint angles
        else:
            self.legState.RR.now_angles = self.get_joint_angles(coord_)
            self.legState.RR.prev_angles = self.legState.RR.now_angles
            self.singularity[2] = False
        return self.legState.RR.now_angles



