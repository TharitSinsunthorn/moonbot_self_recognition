'''
Joint nomeclature:
    joint1: joint near the base
    joint2: joint in-between
    joint3: joint near the tip
'''

L1 = 0.102 # Length between joint1 (Near the base joint) and joint2
L2 = 0.113 # Length between joint2 and joint3 (Near the Tip Joint)
L3 = 0.13 # Length between Joint3 and Tip
D1 = 0.18 # Distance between Origin of base and origin of the joint1

'''
servo_id contains the dynamixel id along with the position of zero angle in the format (id, zero_pos) of three joints of all four limbs
'''

servo_id = {
    "1": [(1, 2017), (2, 2062), (3, 2034)],
    "2": [(4, 2030), (5, 2026), (6, 2009)],
    "3": [(7, 1990), (8, 2052), (9, 2065)],
    "4": [(10, 2073), (11, 2064), (12, 2106)]
}

DIFF_ANGLE = 1023 # The difference between 0 deg and 90 deg

'''
limp_pos contains the angle of the orientation of the limb w.r.t. base frame
'''
limb_angle = [45, 135, -135, -45]

'''
These values are for smoothly controlling the servos
'''
MAX_VELOCITY = 60 # (in deg/s) maximum movement in servo at once.
SLEEP_TIME_SERVO = 0.05 # Time delay between servo commands in seconds.
