'''
Joint nomeclature:
    joint1: joint near the base
    joint2: joint in-between
    joint3: joint near the tip
'''

L1 = 0.065 # Length between joint1 (Near the base joint) and joint2
L2 = 0.125 # Length between joint2 and joint3 (Near the Tip Joint)
L3 = 0.160 # Length between Joint3 and Tip

D1 = 0.240 # Distance between Origin of base and origin of the joint1

'''
servo_id contains the dynamixel id along with the position of zero angle in the format (id, zero_pos) of three joints of all four limbs
'''

servo_id = {
    "1": [(1, 2017), (2, 2062), (3, 2034)],
    "2": [(4, 2030), (5, 2026), (6, 2009)],
    "3": [(7, 1990), (8, 2052), (9, 2065)],
    "4": [(10, 2073), (11, 2064), (12, 2106)]
}


'''
limp_pos contains the angle of the orientation of the limb w.r.t. base frame
'''
limb_angle = [45, 135, -135, -45]

'''
These values are for smoothly controlling the servos
'''
MAX_VELOCITY = 60 # (in deg/s) maximum movement in servo at once.
SLEEP_TIME_SERVO = 0.05 # Time delay between servo commands in seconds.


# Motion related
sec = 1.0
span = 0.13
foward = -0.07
height = 0.23
