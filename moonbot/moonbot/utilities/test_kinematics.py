from limb_kinematics import forward_kinematics, inverse_kinematics
import math
angle1 = 48
angle2 = -90.0
angle3 = 0.0
angle1 = [angle1, angle2, angle3]

position1 = forward_kinematics([36.069, -90.0, 0], 1)
position2 = forward_kinematics([-57.42,-90.0, 0], 2)
position3 = forward_kinematics([48.312, -90.0, 0.0], 3)
position4 = forward_kinematics([-38.43, -90.0, 0.0], 4)
print(position1)
dz = 0
dx = 0.1129
dz1 = -0.00386
dz2 = -0.00386
dz3 = -0.00386
dz4 = -0.00386
position1[0] -= dx
position1[2] -= dz1
position2[0] -= dx
position2[2] -= dz2
position3[0] -= dx
position3[2] -= dz3
position4[0] -= dx
position4[2] -= dz4
print("Initial Angles", [angle1, angle2, angle3])
try:
    angle1 = inverse_kinematics(position1, 1)
    print("First Angle", angle1)
except:
    print("First angle failed")
try:
    angle2 = inverse_kinematics(position2, 2)
    print("Second Angle", angle2)
except:
    print("Seconf angle failed")
try:
    angle3 = inverse_kinematics(position3, 3)
    print("Third Angle", angle3)
except:
    print("thirdd angle failed")

try:
    angle4 = inverse_kinematics(position4, 4)
    print("Forth Angle", angle4)
except:
    print("fourth angle failed")

print(forward_kinematics(angle1, 1))
print(forward_kinematics(angle2, 2))
print(forward_kinematics(angle3, 3))
print(forward_kinematics(angle4, 4))

# print(angle1, angle2, angle3, angle4)
