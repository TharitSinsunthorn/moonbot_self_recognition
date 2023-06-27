from limb_kinematics import forward_kinematics, inverse_kinematics
import math

angle1 = -math.pi/3
angle2 = -math.pi/3
angle3 = -math.pi/3

print([angle1, angle2, angle3])

position = forward_kinematics([angle1, angle2, angle3], 1)
print(inverse_kinematics(position, 1))

