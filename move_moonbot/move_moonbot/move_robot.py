import keyboard
import rclpy
import math
from rclpy.node import Node
import time
from moonbot_custom_interfaces.srv import SetJointAngles


# Note that all joint3 angles are published in opposite angles. This is because of some problem with servo connection or idk.

HOME_POS = [0.0, 30.0, 30.0]
LIMB = [[15.395668506020229, 3.643952495843621, 21.658899255364986], [10.069114418394975, -43.55762527119134, -88.8445378622273], [-10.069114418394975, -43.55762527119134, -88.8445378622273], [-15.395668506020229, 3.643952495843621, 21.658899255364986]]
RESET1 = [[89.850039491688, -9.219923466449245, 47.13538874427036], [1.0735163900000089, -9.884792400462175, 47.21419678757934], [1.0735163900000089, -31.22401100000002, 19.220637800000077]]
RESET2 = [[26.96317036234035, 6.113017998637929, 9.28652581145002], [1.0735163900000089, -9.884792400462175, 47.21419678757934], [1.0735163900000089, -31.22401100000002, 19.220637800000077]]
RESET3 = [[-26.10265920664665, 6.3449483593653895, 10.209450233083203], [1.0735163900000089, -9.884792400462175, 47.21419678757934], [1.0735163900000089, -31.22401100000002, 19.220637800000077]]
RESET4 = [[-89.84429653467441, -10.803821897784246, 47.30198957255925], [1.0735163900000089, -9.884792400462175, 47.21419678757934], [1.0735163900000089, -31.22401100000002, 19.220637800000077]]

class MoveRobot(Node):
    def __init__(self):
        super().__init__('move_robot')
        self.clients_ = [self.create_client(SetJointAngles, f'set_joint_angles_l{i + 1}') for i in range(4)]
        for i in range(4):
            while not self.clients_[i].wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Service not available for Limb {i+1}, waiting ...')
        self.client_futures = []
        # self.moveX()   
         
        self.set_initial_position()
        # self.moveX2()
        # self.test_motion()
        # self.moveX3(RESET1, 1)
        # self.moveX3(RESET2, 2)
        # self.moveX3(RESET3, 3)
        # self.moveX3(RESET4, 4)  

    def test_motion(self):
        request = SetJointAngles.Request()
        # HOME_POS = [90.0, 0.0, 0.0]
        HOME_POS = [1.07351639, -31.904496248389023, 19.29681535845475]
        request.joint1 = HOME_POS[0]
        request.joint2 = HOME_POS[1]
        request.joint3 = HOME_POS[2]
        future = self.clients_[3].call_async(request)
        rclpy.spin_until_future_complete(self, future)

    def spinFutures(self):
        for f in self.client_futures:
            rclpy.spin_until_future_complete(self, f)
        self.client_futures = []


    def set_initial_position(self):
        self.get_logger().info("Initialising position")
        self.move_track = 0
        request = SetJointAngles.Request()
        request.joint1 = HOME_POS[0]
        request.joint2 = HOME_POS[1]
        request.joint3 = HOME_POS[2]

        self.client_futures = []
        for i in range(4):
            self.client_futures.append(self.clients_[i].call_async(request))
        self.spinFutures()
        self.get_logger().info("Limb POsition Initialised")

    def moveX1(self):
        self.get_logger().info("Moving in X direction")
        self.move_track = 0
        self.client_futures = []

        # Arrange the Limbs that favour motion in X-direction

        # Limb 1 movement
        request = SetJointAngles.Request()
        request.joint1 = 0.0
        request.joint2 = -90.0 + 64.06753
        request.joint3 = 76.5022

        future = self.clients_[0].call_async(request)
        rclpy.spin_until_future_complete(self, future)

        request = SetJointAngles.Request()
        request.joint1 = 36.069
        request.joint2 = -90.0 + 64.06753
        request.joint3 = 76.5022

        future = self.clients_[0].call_async(request)
        rclpy.spin_until_future_complete(self, future)

        request = SetJointAngles.Request()
        request.joint1 = 36.069
        request.joint2 = HOME_POS[1]
        request.joint3 = HOME_POS[2]

        future = self.clients_[0].call_async(request)
        rclpy.spin_until_future_complete(self, future)

        # Limb 2 movement
        request = SetJointAngles.Request()
        request.joint1 = 0.0
        request.joint2 = -90.0 + 64.06753
        request.joint3 = 76.5022

        future = self.clients_[1].call_async(request)
        rclpy.spin_until_future_complete(self, future)

        request = SetJointAngles.Request()
        request.joint1 = -57.42
        request.joint2 = -90.0 + 64.06753
        request.joint3 = 76.5022

        future = self.clients_[1].call_async(request)
        rclpy.spin_until_future_complete(self, future)

        request = SetJointAngles.Request()
        request.joint1 = -57.42
        request.joint2 = HOME_POS[1]
        request.joint3 = HOME_POS[2]

        future = self.clients_[1].call_async(request)
        rclpy.spin_until_future_complete(self, future)

        # Limb 3 movement
        request = SetJointAngles.Request()
        request.joint1 = 0.0
        request.joint2 = -90.0 + 64.06753
        request.joint3 = 76.5022

        future = self.clients_[2].call_async(request)
        rclpy.spin_until_future_complete(self, future)

        request = SetJointAngles.Request()
        request.joint1 = 48.312
        request.joint2 = -90.0 + 64.06753
        request.joint3 = 76.5022

        future = self.clients_[2].call_async(request)
        rclpy.spin_until_future_complete(self, future)

        request = SetJointAngles.Request()
        request.joint1 = 48.312
        request.joint2 = HOME_POS[1]
        request.joint3 = HOME_POS[2]

        future = self.clients_[2].call_async(request)
        rclpy.spin_until_future_complete(self, future)
        # Limb 4 movement
        request = SetJointAngles.Request()
        request.joint1 = 0.0
        request.joint2 = -90.0 + 64.06753
        request.joint3 = 76.5022

        future = self.clients_[3].call_async(request)
        rclpy.spin_until_future_complete(self, future)

        request = SetJointAngles.Request()
        request.joint1 = -38.43
        request.joint2 = -90.0 + 64.06753
        request.joint3 = 76.5022

        future = self.clients_[3].call_async(request)
        rclpy.spin_until_future_complete(self, future)

        request = SetJointAngles.Request()
        request.joint1 = -38.43
        request.joint2 = HOME_POS[1]
        request.joint3 = HOME_POS[2]
        future = self.clients_[3].call_async(request)
        rclpy.spin_until_future_complete(self, future)

    def moveX2(self):

        for i in range(4):
            request = SetJointAngles.Request()
            request.joint1 = LIMB[i][0]
            request.joint2 = LIMB[i][1]
            request.joint3 = LIMB[i][2]
            future = self.client_futures.append(self.clients_[i].call_async(request))
        self.spinFutures()

    
    def moveX3(self, RESET, limb):
        # Resetting position
        # Limb 1 movement
        request = SetJointAngles.Request()
        request.joint1 = RESET[0][0]
        request.joint2 = RESET[0][1]
        request.joint3 = RESET[0][2]

        future = self.clients_[limb - 1].call_async(request)
        rclpy.spin_until_future_complete(self, future)

        request = SetJointAngles.Request()
        request.joint1 = RESET[1][0]
        request.joint2 = RESET[1][1]
        request.joint3 = RESET[1][2]

        future = self.clients_[limb - 1].call_async(request)
        rclpy.spin_until_future_complete(self, future)

        request = SetJointAngles.Request()
        request.joint1 = RESET[2][0]
        request.joint2 = RESET[2][1]
        request.joint3 = RESET[2][2]

        future = self.clients_[limb - 1].call_async(request)
        rclpy.spin_until_future_complete(self, future)

def main(args = None):
    rclpy.init(args = args)
    node1 = MoveRobot()
    rclpy.spin(node1)

    node1.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


