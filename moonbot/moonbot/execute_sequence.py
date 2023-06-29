import rclpy
from rclpy.node import Node
from moonbot_custom_interfaces.msg import JointAngles
from moonbot_custom_interfaces.srv import GetJointAngles
import moonbot.utilities.params as params
import time
import numpy as np
'''
This Node reads the leg tip position, perform the inverse kinematics and them publishes the joint angles data.
'''
class ExecuteSequence(Node):
    def __init__(self):
        super().__init__('test_joint_controller')
        self.declare_parameter('limb_num', 2)
        self.limb_num = self.get_parameter('limb_num').get_parameter_value().integer_value
        self.get_logger().info(f"Execute Seqiemce Node Launched for Limb {self.limb_num}")
        self.publisher_angles = self.create_publisher(JointAngles, f'target_joint_angles_l{self.limb_num}', 10)
        self.sequence = [(0.0,0.0,0.0), (45.0, 45.0, 0.0), (0.0,0.0,0.0)]
        self.client_ = self.create_client(GetJointAngles, f'get_joint_angles_{self.limb_num}')
    
    def compute_angle_steps(self, curr_angles, target_angles):
        self.get_logger().info(f'Current Joint Angles for Limb {str(self.limb_num)}: {str(curr_angles)}')
        max_diff = np.max(np.abs(target_angles - curr_angles))
        n_step = int(max_diff/params.MAX_ANGLE_CHANGE)
        angle_steps = np.linspace(curr_angles, target_angles, n_step)
        for i in range(1, n_step):
            msg = JointAngles()
            msg.joint1 = angle_steps[i][0]
            msg.joint2 = angle_steps[i][1]
            msg.joint3 = angle_steps[i][2]
            self.publisher_angles.publish(msg)
            self.get_logger().info(f'Target Joint Angles for Limb {str(self.limb_num)}: {str(angle_steps[i])}')
            time.sleep(params.SLEEP_TIME_SERVO)
        msg = JointAngles()
        msg.joint1 = target_angles[0]
        msg.joint2 = target_angles[1]
        msg.joint3 = target_angles[2]
        self.publisher_angles.publish(msg)
        self.get_logger().info(f'Target Joint Angles for Limb {str(self.limb_num)}: {str(target_angles)}')

    def execute_sequence(self):
        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting ...')

        for item in self.sequence:
            request = GetJointAngles.Request()
            future = self.client_.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            curr_angles = np.array([response.joint1, response.joint2, response.joint3])
            target_angles = np.array(item)
            self.compute_angle_steps(curr_angles, target_angles)

def main(args = None):
    rclpy.init(args = args)
    node1 = ExecuteSequence()
    node1.execute_sequence()
    node1.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


