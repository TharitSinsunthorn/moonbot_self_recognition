import rclpy
import numpy as np
from rclpy.node import Node
import moonbot.utilities.params as params
import time
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
        self.declare_parameter('limb_num', None)
        self.limb_num = self.get_parameter('limb_num').get_parameter_value().integer_value
        self.get_logger().info(f"Joint Interface Node Launched for Limb {self.limb_num}")
        self.servo_id = params.servo_id[str(self.limb_num)]
        self.publisher_servo = self.create_publisher(SetPosition, f'set_position', 10)
        self.subscriber_angles = self.create_subscription(JointAngles, f'target_joint_angles_l{self.limb_num}', self.subscriber_callback, 10)
        self.curr_angles = [0,0,0] # To store current joint angles in deg
        
    def publish_angles(self, joint_angles):
        for i in range(3):
            msg = SetPosition()
            msg.id = self.servo_id[i][0]
            INIT_POS = self.servo_id[i][1]
            msg.position = int(INIT_POS - DIFF_ANGLE/90 * joint_angles[i])
            self.publisher_servo.publish(msg)
        
    def compute_angle_steps(self):
        # curr_angles = np.array(self.curr_angles)
        target_angles = np.array(self.target_angles)
        # max_diff = np.max(np.abs(target_angles - curr_angles))
        # n_step = int(max_diff/params.MAX_ANGLE_CHANGE)
        # angle_steps = np.linspace(curr_angles, target_angles, n_step)
        # for i in range(n_step):
        #     self.publish_angles(angle_steps[i])
        #     self.get_logger().info(f'Target Joint Angles for Limb {str(self.limb_num)}: {str(angle_steps[i])}')
        #     time.sleep(params.SLEEP_TIME_SERVO)
        self.publish_angles(target_angles)
        # self.get_logger().info(f'Target Joint Angles for Limb {str(self.limb_num)}: {str(target_angles)}')


    def subscriber_callback(self, joint_msg):
        self.target_angles = [joint_msg.joint1, joint_msg.joint2, joint_msg.joint3]
        self.compute_angle_steps()

def main(args = None):
    rclpy.init(args = args)
    node1 = JointInterface()
    rclpy.spin(node1)
    node1.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


