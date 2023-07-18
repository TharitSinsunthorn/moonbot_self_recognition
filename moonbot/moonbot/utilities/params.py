'''
Joint nomeclature:
    joint1: joint near the base
    joint2: joint in-between
    joint3: joint near the tip
'''

L1 = 0.102 # Length between joint1 (Near the base joint) and joint2
L2 = 0.113 # Length between joint2 and joint3 (Near the Tip Joint)
L3 = 0.16 # Length between Joint3 and Tip
D1 = 0.18 # Distance between Origin of base and origin of the joint1

'''
servo_id contains the dynamixel id along with the position of zero angle in the format (id, zero_pos) of three joints of all four limbs
'''

servo_id = {
    "1": [(1, 1952), (2, 2070), (3, 2083)],
    "2": [(4, 2059), (5, 2044), (6, 2077)],
    "3": [(7, 2048), (8, 2053), (9, 2065)],
    "4": [(10, 2033), (11, 2064), (12, 2085)]
}


DIFF_ANGLE = 1023 # The difference between 0 deg and 90 deg

'''
limp_pos contains the angle of the orientation of the limb w.r.t. base frame
'''
limb_angle = [45, 135, -135, -45]

'''
These values are for smoothly controlling the servos
'''
MAX_ANGLE_CHANGE = 5 # (in deg) maximum movement in servo at once.
SLEEP_TIME_SERVO = 0.15 # Time delay between servo commands in seconds.
