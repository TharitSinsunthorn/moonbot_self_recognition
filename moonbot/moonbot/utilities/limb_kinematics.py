import moonbot.utilities.params as params
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

def inverse_kinematics(position, leg_num):
    ''' Returns the Inverse Kinematics of the Limb

    Args:
        position: an array containing the x,y,z position (in m) of the limb tip in frame of reference of Base
        leg_num: leg number between 1 to 4
    Returns:
        an array containing the joint angle value (in rad) at each joint.
    '''

    xb, yb, zb = position

    '''
    Coordinate Transformation: Base to Joint 1 Frame
    '''
    theta = params.limb_angle[leg_num-1]

    #### Translation ####
    xb = xb - params.D1 * math.cos(theta)
    yb = yb - params.D1 * math.sin(theta)


    ### Rotation
    
    x = xb * math.cos(theta) + yb * math.sin(theta)
    y = -xb * math.sin(theta) + yb * math.cos(theta)
    z = zb

    ### Inverse Kinematics

    LL = np.hypot(x, y)
    D = np.sqrt(z**2 + (LL-L1)**2)

    th1 = np.arctan2(y, x)
    th2 = np.arctan2(z, (LL-L1)) - np.arccos((L3**2 - L2**2 - D**2)/(-2*D*L2))
    th3 = math.pi - np.arccos((D**2 - L2**2 - L3**2)/(-2*L2*L3))

    return [th1, th2, th3]

def forward_kinematics(joint_angles, leg_num):
    ''' Returns the Forward Kinematics of the Limb

    Args:
        joint_angles: an array containing the joint angle value (in rad) at each joint
        leg_num: leg number between 1 to 4
    Returns:
        an array containing the x,y,z position (in m) of the limp tip in frame of reference of Base
    '''
    Th1, Th2, Th3 = joint_angles

    ### Forward Kinematics

    x1 = L1*math.cos(Th1)
    y1 = L1*math.sin(Th1)
    z1 = 0.0

    x2 = x1 + L2*math.cos(Th2)*math.cos(Th1)
    y2 = y1 + L2*math.cos(Th2)*math.sin(Th1)
    z2 = z1 + L2*math.sin(Th2)

    x3 = x2 + L3*math.cos(Th2+Th3)*math.cos(Th1)
    y3 = y2 + L3*math.cos(Th2+Th3)*math.sin(Th1)
    z3 = z2 + L3*math.sin(Th2+Th3)

    '''
    Coordinate Transformation: Joint 1 to Base Frame
    '''
    theta = params.limb_angle[leg_num-1]

    ### Rotation
    
    x = x3 * math.cos(theta) - y3 * math.sin(theta)
    y = x3 * math.sin(theta) + y3 * math.cos(theta)
    z = z3

    #### Translation ####
    x = x + params.D1 * math.cos(theta)
    y = y + params.D1 * math.sin(theta)

    return [x, y, z]





