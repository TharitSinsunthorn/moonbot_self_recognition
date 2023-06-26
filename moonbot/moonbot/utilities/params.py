'''
Joint nomeclature:
    joint1: joint near the base
    joint2: joint in-between
    joint3: joint near the tip
'''

L1 = 1.5 # Length between joint1 (Near the base joint) and joint2
L2 = 4 # Length between joint2 and joint3 (Near the Tip Joint)
L3 = 2 # Length between Joint3 and Tip
D1 = 0 # Distance between Origin of base and origin of the joint1

'''
servo_id contains the dynamixel of three joints of all four limbs
'''

servo_id = {
    "1": [1, 2, 3],
    "2": [1, 2, 3],
    "3": [1, 2, 3],
    "4": [1, 2, 3]
}

'''
limp_pos contains the angle of the orientation of the limb w.r.t. base frame
'''
limb_angle = [0, 135, -135, -45]