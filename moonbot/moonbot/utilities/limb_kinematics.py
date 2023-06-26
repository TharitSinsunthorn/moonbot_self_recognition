try:
    import moonbot.utilities.params as params
except:
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
    
    print(x, y, z, LL, D)
    
    th1 = np.arctan2(y,x)
    th2 = math.pi/2 - np.arctan2((LL-L1), z)- np.arccos((L3**2 - L2**2 - D**2)/(-2*D*L2))
    th3 = np.arccos((D**2 - L2**2 - L3**2)/(2*L2*L3))

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

    x = math.cos(Th1)* (L1 + L2*math.cos(Th2) + L3*math.cos(Th2+Th3))
    y = math.sin(Th1)* (L1 + L2*math.cos(Th2) + L3*math.cos(Th2+Th3))
    z = L2*math.sin(Th2) + L3*math.sin(Th2+Th3)

    '''
    Coordinate Transformation: Joint 1 to Base Frame
    '''
    theta = params.limb_angle[leg_num-1]

    ### Rotation
    
    xb = x * math.cos(theta) - y * math.sin(theta)
    yb = x * math.sin(theta) + y * math.cos(theta)
    zb = z

    #### Translation ####
    xb = xb + params.D1 * math.cos(theta)
    yb = yb + params.D1 * math.sin(theta)

    return [xb, yb, zb]





