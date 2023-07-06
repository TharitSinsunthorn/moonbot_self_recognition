from limb_kinematics import forward_kinematics, inverse_kinematics
import math
angle1 = 0
angle2 = -25
angle3 = -75
angle = [angle1, angle2, angle3]

position1 = forward_kinematics(angle, 1)
position2 = forward_kinematics(angle, 2)
position3 = forward_kinematics(angle, 3)
position4 = forward_kinematics(angle, 4)
print(position1)
dz = 0
dx = 0.172
position1[0] += dx * math.cos(3.14/4)
position1[1] += dx * math.cos(3.14/4)
position1[2] -= dz
position2[0] += dx * math.cos(3.14/4)
position2[1] += dx * math.cos(3.14/4)
position2[2] -= dz
position3[0] += dx * math.cos(3.14/4)
position3[1] += dx * math.cos(3.14/4)
position3[2] -= dz
position4[0] += dx * math.cos(3.14/4)
position4[1] += dx * math.cos(3.14/4)
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

print(forward_kinematics(angle1, 1))
print(forward_kinematics(angle2, 2))
# print(angle1, angle2, angle3, angle4)
