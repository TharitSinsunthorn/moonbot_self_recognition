import rclpy
import math
from rclpy.node import Node
import moonbot.utilities.params as params

from moonbot_custom_interfaces.msg import JointAngles
from moonbot_custom_interfaces.msg import SetPosition

'''
This Node acts as interface between joint_angle topic and dynamixel control code.
Note that joint angle must be within the range of -90 to 90 degs
'''
DIFF_ANGLE = params.DIFF_ANGLE

class JointInterface(Node):
    def __init__(self):
        super().__init__('joint_interface')
        self.declare_parameter('limb_num', 1)
        self.limb_num = self.get_parameter('limb_num').get_parameter_value().integer_value
        self.servo_id = params.servo_id[str(self.limb_num)]
        self.publisher_servo = self.create_publisher(SetPosition, f'set_position', 10)
        self.subcriber_angles = self.create_subscription(JointAngles, f'target_joint_angles_l{self.limb_num}', self.subscriber_callback, 10)


    def subscriber_callback(self, joint_msg):
        msg = SetPosition()
        msg.id = self.servo_id[0][0]
        INIT_POS = self.servo_id[0][1]
        msg.position = int(INIT_POS - DIFF_ANGLE/(math.pi/2) * joint_msg.joint1)
        self.publisher_servo.publish(msg)

        msg = SetPosition()
        msg.id = self.servo_id[1][0]
        INIT_POS = self.servo_id[1][1]
        msg.position = int(INIT_POS - DIFF_ANGLE/(math.pi/2) * joint_msg.joint2)
        self.publisher_servo.publish(msg)

        msg = SetPosition()
        msg.id = self.servo_id[2][0]
        INIT_POS = self.servo_id[2][1]
        msg.position = int(INIT_POS - DIFF_ANGLE/(math.pi/2) * joint_msg.joint3)
        self.publisher_servo.publish(msg)

def main(args = None):
    rclpy.init(args = args)
    node1 = JointInterface()
    rclpy.spin(node1)

    node1.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


