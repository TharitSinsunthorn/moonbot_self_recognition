import rclpy
import math
from rclpy.node import Node
from moonbot.utilities.limb_kinematics import inverse_kinematics

from moonbot_custom_interfaces.msg import JointAngles
from geometry_msgs.msg import Point 

'''
This Node reads the leg tip position, perform the inverse kinematics and them publishes the joint angles data.
'''
class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')
        self.declare_parameter('limb_num', None)
        self.limb_num = self.get_parameter('limb_num').get_parameter_value().integer_value
        self.get_logger().info(f"Joint Controller Node Launched for Limb {self.limb_num}")
        self.publisher_angles = self.create_publisher(JointAngles, f'target_joint_angles_l{self.limb_num}', 10)
        self.subcriber_positions = self.create_subscription(Point, f'target_tip_position_l{self.limb_num}', self.subscriber_callback, 10)
    
    def subscriber_callback(self, msg):
        self.get_logger().info(f"New Position for Limb: {self.limb_num}: {[msg.x, msg.y, msg.z]}")
        position = [msg.x, msg.y, msg.z]
        angles = JointAngles()
        angles.joint1, angles.joint2, angles.joint3 = inverse_kinematics(position, self.limb_num)
        self.get_logger().info(f"New Joint Angles for Limb: {self.limb_num}: {[angles.joint1, angles.joint2, angles.joint3]}")
        self.publisher_angles.publish(angles)

def main(args = None):
    rclpy.init(args = args)
    node1 = JointController()
    rclpy.spin(node1)

    node1.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


