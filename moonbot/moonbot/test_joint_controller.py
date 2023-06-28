import rclpy
import math
from rclpy.node import Node
from moonbot_custom_interfaces.msg import JointAngles

'''
This Node reads the leg tip position, perform the inverse kinematics and them publishes the joint angles data.
'''
class TestJointController(Node):
    def __init__(self):
        super().__init__('test_joint_controller')
        self.publisher_angles1 = self.create_publisher(JointAngles, f'target_joint_angles_l1', 10)
        self.timer = self.create_timer(0.15, self.timer_clbk)
        self.dt = 5
        self.angles = 0
    
    def timer_clbk(self):
        angles1 = JointAngles()
        angles1.joint1 = (math.sin(math.radians(self.angles)) + 1) * 40
        angles1.joint2 = (math.sin(math.radians(self.angles + 45)) + 1) * 40
        angles1.joint3 = (math.sin(math.radians(self.angles + 90)) + 1) * 40
        self.publisher_angles1.publish(angles1)
        self.angles += self.dt

def main(args = None):
    rclpy.init(args = args)
    node1 = TestJointController()
    rclpy.spin(node1)

    node1.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


