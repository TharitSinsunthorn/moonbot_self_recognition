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
servo_id contains the dynamixel id along with the position of zero angle in the format (id, zero_pos) of three joints of all four limbs
'''

servo_id = {
    "1": [(3, 2060), (2, 3103), (1, 2052)],
    "2": [(4, 2062), (5, 2039), (6, 2070)],
    "3": [1, 2, 3],
    "4": [1, 2, 3]
}

DIFF_ANGLE = 1023 # The difference between 0 deg and 90 deg

'''
limp_pos contains the angle of the orientation of the limb w.r.t. base frame
'''
limb_angle = [0, 135, -135, -45]

'''
These values are for smoothly controlling the servos
'''
MAX_ANGLE_CHANGE = 5 # (in deg) maximum movement in servo at once.
SLEEP_TIME_SERVO = 0.2 # Time delay between servo commands in seconds.
