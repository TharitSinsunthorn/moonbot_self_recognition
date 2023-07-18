import keyboard
import rclpy
import math
from rclpy.node import Node
import time
from moonbot_custom_interfaces.srv import SetJointAngles


# Note that all joint3 angles are published in opposite angles. This is because of some problem with servo connection or idk.
LIMB = [[85.85373380239122, 14.903879745529537, 53.55957100573028], [26.739285483242554, -15.746146939759456, -54.84569706779632],
                [-24.686857075675107, -9.711893881130436, -43.573846044136985], [-85.45721490362689, 11.379671210797994, 56.041353402891104]]
HOME_POS = [2.4382, 7.145, 45.321]
class MoveRobot(Node):
    def __init__(self):
        super().__init__('move_robot')
        self.clients_ = [self.create_client(SetJointAngles, f'set_joint_angles_l{i + 1}') for i in range(4)]
        for i in range(4):
            while not self.clients_[i].wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Service not available for Limb {i+1}, waiting ...')
        self.client_futures = []
        # self.moveX()   
        # self.test_motion() 
        # self.set_initial_position()
        # self.moveX1()
        self.moveX2()
        # self.moveX3()
        # self.moveX1()
        # self.moveX2()
        # self.moveX3()  
        # self.moveX1()
        # self.moveX2()
        # self.moveX3()      

    def test_motion(self):
        request = SetJointAngles.Request()
        # HOME_POS = [90.0, 0.0, 0.0]
        HOME_POS = [-90.0, 90.0, 0.0]
        request.joint1 = HOME_POS[0]
        request.joint2 = HOME_POS[1]
        request.joint3 = HOME_POS[2]
        future = self.clients_[1].call_async(request)
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

    
    def moveX3(self):
        # Resetting position
        # Limb 1 movement
        request = SetJointAngles.Request()
        request.joint1 = LIMB[0][0]
        request.joint2 = -90.0 + 64.06753
        request.joint3 = 76.5022

        future = self.clients_[0].call_async(request)
        rclpy.spin_until_future_complete(self, future)

        request = SetJointAngles.Request()
        request.joint1 = 0.0
        request.joint2 = -90.0 + 64.06753
        request.joint3 = 76.5022

        future = self.clients_[0].call_async(request)
        rclpy.spin_until_future_complete(self, future)

        request = SetJointAngles.Request()
        request.joint1 = HOME_POS[0]
        request.joint2 = HOME_POS[1]
        request.joint3 = HOME_POS[2]

        future = self.clients_[0].call_async(request)
        rclpy.spin_until_future_complete(self, future)

        # # Limb 2 movement
        # request = SetJointAngles.Request()
        # request.joint1 = LIMB[1][0]
        # request.joint2 = -90.0 + 64.06753
        # request.joint3 = 76.5022

        # future = self.clients_[1].call_async(request)
        # rclpy.spin_until_future_complete(self, future)

        # request = SetJointAngles.Request()
        # request.joint1 = 0.0
        # request.joint2 = -90.0 + 64.06753
        # request.joint3 = 76.5022

        # future = self.clients_[1].call_async(request)
        # rclpy.spin_until_future_complete(self, future)

        request = SetJointAngles.Request()
        request.joint1 = HOME_POS[0]
        request.joint2 = HOME_POS[1]
        request.joint3 = HOME_POS[2]

        future = self.clients_[1].call_async(request)
        rclpy.spin_until_future_complete(self, future)

        # # Limb 3 movement
        # request = SetJointAngles.Request()
        # request.joint1 = LIMB[2][0]
        # request.joint2 = -90.0 + 64.06753
        # request.joint3 = 76.5022

        # future = self.clients_[2].call_async(request)
        # rclpy.spin_until_future_complete(self, future)

        # request = SetJointAngles.Request()
        # request.joint1 = 0.0
        # request.joint2 = -90.0 + 64.06753
        # request.joint3 = 76.5022

        # future = self.clients_[2].call_async(request)
        # rclpy.spin_until_future_complete(self, future)

        request = SetJointAngles.Request()
        request.joint1 = HOME_POS[0]
        request.joint2 = HOME_POS[1]
        request.joint3 = HOME_POS[2]

        future = self.clients_[2].call_async(request)
        rclpy.spin_until_future_complete(self, future)

        # Limb 4 movement
        request = SetJointAngles.Request()
        request.joint1 = LIMB[3][0]
        request.joint2 = -90.0 + 64.06753
        request.joint3 = 76.5022

        future = self.clients_[3].call_async(request)
        rclpy.spin_until_future_complete(self, future)

        request = SetJointAngles.Request()
        request.joint1 = 0.0
        request.joint2 = -90.0 + 64.06753
        request.joint3 = 76.5022

        future = self.clients_[3].call_async(request)
        rclpy.spin_until_future_complete(self, future)

        request = SetJointAngles.Request()
        request.joint1 = HOME_POS[0]
        request.joint2 = HOME_POS[1]
        request.joint3 = HOME_POS[2]

        future = self.clients_[3].call_async(request)
        rclpy.spin_until_future_complete(self, future)

        self.get_logger().info("Main Body Moved in X Direction")

def main(args = None):
    rclpy.init(args = args)
    node1 = MoveRobot()
    rclpy.spin(node1)

    node1.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


