import rclpy
import math
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
        self.declare_parameter('limb_num', 1)
        self.limb_num = self.get_parameter('limb_num').get_parameter_value().integer_value
        self.servo_id = params.servo_id[str(self.limb_num)]
        self.publisher_servo = self.create_publisher(SetPosition, f'set_position', 10)
        self.subcriber_angles = self.create_subscription(JointAngles, f'target_joint_angles_l{self.limb_num}', self.subscriber_callback, 10)
        self.client_ = self.create_client(GetPosition, 'get_position')
        self.curr_angles = [0,0,0]
        self.target_angles = [0,0,0]
        self.completed_requests = 0
        self.rate = self.create_rate(1/params.SLEEP_TIME_SERVO)
    
    def update_values(self, joint_num):
        if not self.client_ .wait_for_service(timeout_sec=1.0):
            self.get_logger.warn('Service not available')
            return
        request = GetPosition.Request()
        request.id = self.servo_id[joint_num][0]
        future = self.client_.call_async(request)
        clbk_function = lambda f: self.handle_response(f, joint_num)
        future.add_done_callback(clbk_function)
    
    def handle_response(self, future, joint_num):
        try:
            response = future.result()
            position = response.position
            INIT_POS = self.servo_id[joint_num][1]
            self.curr_angles[joint_num] = (INIT_POS - position) * 90 / DIFF_ANGLE
            self.completed_requests += 1

        except Exception as e:
            self.get_logger().info('Additional data: %s'%str(e))
            self.update_values(joint_num)
        
        if self.completed_requests == 3:
            self.completed_requests = 0
            self.get_logger().info(f"Current Joint Angles for Limb {str(self.limb_num)}: {str(self.curr_angles)}")
            self.compute_angle_steps()
    
    def publish_angles(self, joint_angles):
        for i in range(3):
            msg = SetPosition()
            msg.id = self.servo_id[i][0]
            INIT_POS = self.servo_id[i][1]
            msg.position = int(INIT_POS - DIFF_ANGLE/90 * joint_angles[i])
            self.publisher_servo.publish(msg)
        
    def compute_angle_steps(self):
        curr_angles = np.array(self.curr_angles)
        target_angles = np.array(self.target_angles)

        max_diff = np.max(np.abs(target_angles - curr_angles))
        n_step = int(max_diff/params.MAX_ANGLE_CHANGE)
        angle_steps = np.linspace(curr_angles, target_angles, n_step)
        for i in range(n_step):
            self.publish_angles(angle_steps[i])
            self.get_logger().info(f'Target Joint Angles for Limb {str(self.limb_num)}: {str(angle_steps[i])}')
            time.sleep(params.SLEEP_TIME_SERVO)
        self.publish_angles(target_angles)
        self.get_logger().info(f'Target Joint Angles for Limb {str(self.limb_num)}: {str(target_angles)}')


    def subscriber_callback(self, joint_msg):
        self.target_angles = [joint_msg.joint1, joint_msg.joint2, joint_msg.joint3]
        for i in range(3):
            self.update_values(i)

def main(args = None):
    rclpy.init(args = args)
    node1 = JointInterface()
    rclpy.spin(node1)
    node1.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


