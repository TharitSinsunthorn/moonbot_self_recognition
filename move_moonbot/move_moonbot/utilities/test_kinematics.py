from limb_kinematics import forward_kinematics, inverse_kinematics
import math

angle1 = 1
angle2 = 48
angle3 = -67.9561
angle = [angle1, angle2, angle3]


position1 = forward_kinematics(angle, 1)
position2 = forward_kinematics(angle, 2)
position3 = forward_kinematics(angle, 3)
position4 = forward_kinematics(angle, 4)
dz = 0.027
dx = 0.17095
position1[0] += dx
position1[2] -= dz
position2[0] += dx
position2[2] -= dz
position3[0] += dx
position3[2] -= dz
position4[0] += dx
position4[2] -= dz

try:
    angle1 = inverse_kinematics(position1, 1)
    print(angle1)
except:
    print("First angle failed")
try:
    angle2 = inverse_kinematics(position2, 2)
    print(angle2)
except:
    print("Seconf angle failed")
try:
    angle3 = inverse_kinematics(position3, 3)
    print(angle3)
except:
    print("thirdd angle failed")

try:
    angle4 = inverse_kinematics(position4, 4)
    print(angle4)
except:
    print("fourth angle failed")
# print(angle1, angle2, angle3, angle4)
