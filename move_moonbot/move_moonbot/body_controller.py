import rclpy
import math
from rclpy.node import Node
from moonbot_custom_interfaces.srv import GetPosition
from geometry_msgs.msg import Point
import moonbot.utilities.params as params
from moonbot.utilities.limb_kinematics import forward_kinematics as FK
from geometry_msgs.msg import Point 

'''
This Node performs the gait analysis and publishes the limb tip position for all the limbs
'''
class BodyController(Node):
    def __init__(self):
        super().__init__('body_controller')
        self.publisher_pos = [self.create_publisher(Point, f'target_tip_position_l{i+1}', 10) for i in range(4)]
        self.legTipPos = [(0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)]
        self.client_ = [self.create_client(GetPosition, f'get_position_d{i + 1}') for i in range(2)]
        while not self.client_[0].wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting ...')
        while not self.client_[1].wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting ...')
        self.moveX()

    def updateLegTipPos(self):
        for k in range(2):
            for j in range(2):
                limb_num = 2 * k + j + 1
                servo_id = params.servo_id[str(limb_num)]
                curr_angles = [0,0,0]
                pos_arr = [0,0,0]
                for i in range(3):
                    request = GetPosition.Request()
                    request.id = servo_id[i][0]
                    INIT_POS = servo_id[i][1]
                    future = self.client_[k].call_async(request)
                    rclpy.spin_until_future_complete(self, future)
                    response = future.result()
                    position = response.position
                    pos_arr[i] = position
                    curr_angles[i] = (INIT_POS - position) * 90 / params.DIFF_ANGLE
                self.legTipPos[limb_num - 1] = FK(curr_angles, limb_num)
                self.get_logger().info(f"Position of Limb {limb_num}: {self.legTipPos[limb_num - 1]}")
    def moveX(self):
        velocityX = 0.05
        velocityY = 0
        velocityZ = 0
        self.updateLegTipPos()
        for i in range(4):
            new_position = Point()
            new_position.x = self.legTipPos[i][0] - velocityX
            new_position.y = self.legTipPos[i][1] - velocityY
            new_position.z = self.legTipPos[i][2] - velocityZ
            # self.publisher_pos[i].publish(new_position)
        

def main(args = None):
    rclpy.init(args = args)
    node1 = BodyController()
    rclpy.spin(node1)

    node1.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


