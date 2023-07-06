import rclpy
import numpy as np
from rclpy.node import Node
import moonbot.utilities.params as params
import time
from moonbot_custom_interfaces.msg import JointAngles
from moonbot_custom_interfaces.msg import SetPosition
from moonbot_custom_interfaces.srv import GetPosition

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
        self.client_ = self.create_client(GetPosition, f'get_position')
        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting ...')
        self.curr_angles = np.array([0,0,0]) # To store current joint angles in deg
        self.sequence = [(0.0, -90.0, 0.0)]
        if self.limb_num == 1:
            self.sequence = [(0.0, -90.0, 0.0), (0.0, 90.0, 0.0), (45.0, 90.0, 0.0), (-45.0, 90.0, 0.0), (45.0, 90.0, 0.0), (-45.0, 90.0, 0.0), (0.0, 90.0, 0.0), (0.0, 90.0, 90.0), (0.0, -90.0, 0.0)]
        if self.limb_num == 2:
            self.sequence = [(-45.0, -90.0, 0.0)]
        if self.limb_num == 4:
            self.sequence = [(45.0, -90.0, 0.0)]
        self.empty_sequence = True
        self.update_curr_pos()
        self.execute_sequence()


    def compute_angle_steps(self, curr_angles, target_angles):
        self.get_logger().info(f'Current Joint Angles for Limb {str(self.limb_num)}: {str(curr_angles)}')
        max_diff = np.max(np.abs(target_angles - curr_angles))
        n_step = int(max_diff/params.MAX_ANGLE_CHANGE)
        angle_steps = np.linspace(curr_angles, target_angles, n_step)
        for i in range(1, n_step):
            self.publish_angles(angle_steps[i])
            self.get_logger().info(f'Target Joint Angles for Limb {str(self.limb_num)}: {str(angle_steps[i])}')
            time.sleep(params.SLEEP_TIME_SERVO)
        self.publish_angles(target_angles)
        self.curr_angles = target_angles
        self.get_logger().info(f'Target Joint Angles for Limb {str(self.limb_num)}: {str(target_angles)}')


    def update_curr_pos(self):
        for i in range(3):
            request = GetPosition.Request()
            request.id = self.servo_id[i][0]
            INIT_POS = self.servo_id[i][1]
            future = self.client_.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            position = response.position
            self.curr_angles[i] = (INIT_POS - position) * 90 / params.DIFF_ANGLE
        self.get_logger().info(f'Current Position Updated for Limb {str(self.limb_num)}: {self.curr_angles}')
    
    def execute_sequence(self):
        while(len(self.sequence)):
            # self.update_curr_pos() ## Updating curr position here is apparently prohibiting running the future codes.   
            target_angles = np.array(self.sequence.pop(0))
            self.compute_angle_steps(self.curr_angles, target_angles)
        self.empty_sequence = True

        
    def publish_angles(self, joint_angles):
        for i in range(3):
            msg = SetPosition()
            msg.id = self.servo_id[i][0]
            INIT_POS = self.servo_id[i][1]
            msg.position = int(INIT_POS - DIFF_ANGLE/90 * joint_angles[i])
            self.publisher_servo.publish(msg)

    def subscriber_callback(self, joint_msg):
        joint_angles = [joint_msg.joint1, joint_msg.joint2, joint_msg.joint3]
        if (list(self.curr_angles) != joint_angles) :
            self.sequence.append(joint_angles)
            if self.empty_sequence:
                self.empty_sequence = False
                self.execute_sequence()

def main(args = None):
    rclpy.init(args = args)
    node1 = JointInterface()
    rclpy.spin(node1)
    node1.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


