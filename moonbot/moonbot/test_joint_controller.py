import rclpy
import math
from rclpy.node import Node
from moonbot_custom_interfaces.msg import JointAngles
import random
'''
This Node reads the leg tip position, perform the inverse kinematics and them publishes the joint angles data.
'''
class TestJointController(Node):
    def __init__(self):
        super().__init__('test_joint_controller')
        self.publisher_angles1 = self.create_publisher(JointAngles, f'target_joint_angles_l1', 10)
        self.publisher_angles2 = self.create_publisher(JointAngles, f'target_joint_angles_l2', 10)
        self.timer = self.create_timer(2, self.timer_clbk)
    
    def timer_clbk(self):
        angles1 = JointAngles()
        angles1.joint1 = float(random.randint(-90, 90))
        angles1.joint2 = float(random.randint(-90, 80))
        angles1.joint3 = float(random.randint(-90, 90))

        angles2 = JointAngles()
        angles2.joint1 = angles1.joint1
        angles2.joint2 = angles1.joint2
        angles2.joint3 = angles1.joint3

        self.publisher_angles1.publish(angles1)
        self.publisher_angles2.publish(angles2)

def main(args = None):
    rclpy.init(args = args)
    node1 = TestJointController()
    rclpy.spin(node1)

    node1.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


