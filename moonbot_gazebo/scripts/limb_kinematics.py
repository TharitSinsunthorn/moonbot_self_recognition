import params as params
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

L = [L1, L2, L3]

def bound_range(angle):
    angle = angle%360
    angle = (angle + 360) % 360
    if (angle > 180):
        angle-=360
    return angle


def inverse_kinematics(X):
    ''' Returns the Inverse Kinematics of the Limb

    Args:
        position: an array containing the x,y,z position (in m) of the limb tip in frame of reference of Base
        leg_num: leg number between 1 to 4
    Returns:
        an array containing the joint angle value (in deg) at each joint.
    '''

    x, y, z = X
    
    L1 = params.L1
    L2 = params.L2
    L3 = params.L3

    LL = np.hypot(x, y)
    D = np.sqrt(z**2 + (LL-L1)**2)
    
    th1 = np.arctan2(y,x)
    th2 = np.arctan2(z, (LL-L1)) - np.arccos((L3**2 - L2**2 - D**2)/(-2*D*L2))
    th3 = math.pi - np.arccos((D**2 - L2**2 - L3**2)/(-2*L2*L3))

    return [th1, th2, th3] # The negative sign in third angle is because third servo is acting in opposite direction

def forward_kinematics(th):
    ''' Returns the Forward Kinematics of the Limb

    Args:
        joint_angles: an array containing the joint angle value (in deg) at each joint
        leg_num: leg number between 1 to 4
    Returns:
        an array containing the x,y,z position (in m) of the limp tip in frame of reference of Base
    '''
    L1 = params.L1
    L2 = params.L2
    L3 = params.L3
    Th1, Th2, Th3 = th

    Th1 = math.radians(Th1)
    Th2 = math.radians(Th2)
    Th3 = -math.radians(Th3) # The negative sign in third angle is because third servo is acting in opposite direction

    ### Forward Kinematics

    x = math.cos(Th1)* (L1 + L2*math.cos(Th2) + L3*math.cos(Th2+Th3))
    y = math.sin(Th1)* (L1 + L2*math.cos(Th2) + L3*math.cos(Th2+Th3))
    z = L2*math.sin(Th2) + L3*math.sin(Th2+Th3)

    '''
    Coordinate Transformation: Joint 1 to Base Frame
    '''
    theta = math.radians(params.limb_angle[leg_num-1])

    ### Rotation
    
    xb = x * math.cos(theta) - y * math.sin(theta)
    yb = x * math.sin(theta) + y * math.cos(theta)
    zb = z

    #### Translation ####
    xb = xb + params.D1 * math.cos(theta)
    yb = yb + params.D1 * math.sin(theta)

    return [xb, yb, zb]





