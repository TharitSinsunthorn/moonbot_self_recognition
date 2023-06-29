import rclpy
import numpy as np
from rclpy.node import Node
import moonbot.utilities.params as params
import time
from moonbot_custom_interfaces.msg import JointAngles
from moonbot_custom_interfaces.msg import SetPosition
from moonbot_custom_interfaces.msg import DynamixelPosition
from moonbot_custom_interfaces.srv import GetJointAngles

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
        self.subscriber_J1 = self.create_subscription(DynamixelPosition, f'get_position_{self.servo_id[0][0]}', self.subscriber_clbk_J1, 10)
        self.subscriber_J2 = self.create_subscription(DynamixelPosition, f'get_position_{self.servo_id[1][0]}', self.subscriber_clbk_J2, 10)
        self.subscriber_J3 = self.create_subscription(DynamixelPosition, f'get_position_{self.servo_id[2][0]}', self.subscriber_clbk_J3, 10)
        self.joint_angles_srv = self.create_service(GetJointAngles, f'get_joint_angles_{self.limb_num}', self.joint_srv_clbk)
        self.curr_angles = [0,0,0] # To store current joint angles in deg
        
    def joint_srv_clbk(self, request, response):
        response.joint1 = self.curr_angles[0]
        response.joint2 = self.curr_angles[1]
        response.joint3 = self.curr_angles[2]
        self.get_logger().info(f"Incoming request for joint angles for limb: {self.limb_num}")
        return response
    
    def subscriber_clbk_J1(self, msg):
        INIT_POS = self.servo_id[0][1]
        position = msg.position
        self.curr_angles[0] = (INIT_POS - position) * 90 / DIFF_ANGLE
    def subscriber_clbk_J2(self, msg):
        INIT_POS = self.servo_id[1][1]
        position = msg.position
        self.curr_angles[1] = (INIT_POS - position) * 90 / DIFF_ANGLE
    def subscriber_clbk_J3(self, msg):
        INIT_POS = self.servo_id[2][1]
        position = msg.position
        self.curr_angles[2] = (INIT_POS - position) * 90 / DIFF_ANGLE
 
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


