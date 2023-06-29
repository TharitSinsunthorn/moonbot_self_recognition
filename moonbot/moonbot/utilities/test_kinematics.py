from limb_kinematics import forward_kinematics, inverse_kinematics
import math

angle1 = 0
angle2 = math.pi/2
angle3 = 0

print([angle1, angle2, angle3])


angle = [angle1, angle2, angle3]
print("Cross Checking:")
position = forward_kinematics(angle, 1)
print("Position 1:", position)
angle = inverse_kinematics(position, 1)
print("Angles :", angle)
position = forward_kinematics(angle, 1)
print("Position 2:", position)

position[0] -= 0.1

print("Cross Checking:")
print("Position 1:", position)
angle = inverse_kinematics(position, 1)
print("Angles :", angle)
position = forward_kinematics(angle, 1)
print("Position 2:", position)