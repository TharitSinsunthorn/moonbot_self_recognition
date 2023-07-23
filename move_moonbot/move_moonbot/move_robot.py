import keyboard
import rclpy
import math
from rclpy.node import Node
import time
from moonbot_custom_interfaces.srv import SetJointAngles


# Note that all joint3 angles are published in opposite angles. This is because of some problem with servo connection or idk.

HOME_POS1 = [0.0, -80.0, -35.0]
HOME_POS2 = [0.0, -80.0, -35.0]
HOME_POS3 = [0.0, -80.0, -35.0]
HOME_POS4 = [0.0, -80.0, -35.0]
FC1 = [42.75367285946197, -87.56527231978885, -28.818799182685552]
HE1 = [-17.978297559378234, -69.13018151954225, -40.264044618557136]
HEH1 = [-17.978297559378234, -22.44468177418719, -116.00228327659863]
FEH1 = [-32.08994052366563, 3.518957523478093, -102.36615104573639]
FE1 = [-32.08994052366563, -75.73519810712423, 9.132933625423277]
FC2 = [-42.75367285946197, -87.56527231978885, -28.818799182685552]
HE2 = [17.97829755937829, -69.13018151954225, -40.264044618557136]
HEH2 = [17.97829755937829, -22.44468177418719, -116.00228327659863]
FEH2 = [32.08994052366563, 3.518957523478093, -102.36615104573639]
FE2 = [32.08994052366563, -75.73519810712423, 9.132933625423277]
FC3 = [42.75367285946197, -87.56527231978885, -28.818799182685552]
HE3 = [-17.978297559378234, -69.13018151954225, -40.264044618557136]
HEH3 = [-17.978297559378234, -22.44468177418719, -116.00228327659863]
FEH3 = [-32.08994052366563, 3.518957523478093, -102.36615104573639]
FE3 = [-32.08994052366563, -75.73519810712423, 9.132933625423277]
FC4 = [-42.75367285946197, -87.56527231978885, -28.818799182685552]
HE4 = [17.97829755937829, -69.13018151954225, -40.264044618557136]
HEH4 = [17.97829755937829, -22.44468177418719, -116.00228327659863]
FEH4 = [32.08994052366563, 3.518957523478093, -102.36615104573639]
FE4 = [32.08994052366563, -75.73519810712423, 9.132933625423277]

class MoveRobot(Node):
    def __init__(self):
        super().__init__('move_robot')
        self.clients_ = [self.create_client(SetJointAngles, f'set_joint_angles_l{i + 1}') for i in range(4)]
        for i in range(4):
            while not self.clients_[i].wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Service not available for Limb {i+1}, waiting ...')
        self.client_futures = []
        # self.moveX()
        self.setPositions([(HOME_POS1, 1), (HOME_POS2, 2), (HOME_POS3, 3), (HOME_POS4, 4)])
        self.setPositions([(HE1, 1), (FC2, 2), (FC3, 3), (HE4, 4)]) 
        # self.setPositions([(HEH1, 1)])
        # self.setPositions([(FEH1, 1)])
        # self.setPositions([(FE1, 1)])
        # self.setPositions([(HEH4, 4)])
        # self.setPositions([(FEH4, 4)])
        # self.setPositions([(FE4, 4)])
        # self.setPositions([(FC1, 1), (FC4, 4), (FE3, 3), (FE2, 2)])
        # self.setPositions([(FEH2, 2)])
        # self.setPositions([(HEH2, 2)])
        # self.setPositions([(HE2, 2)])
        # self.setPositions([(FEH3, 3)])
        # self.setPositions([(HEH3, 3)])
        # self.setPositions([(HE3, 3)])
        # self.setPositions([(HOME_POS1, 1), (HOME_POS2, 2), (HOME_POS3, 3), (HOME_POS4, 4)])


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

    def setPositions(self, limbPositions):
        '''
        limbPositions: [(POS, legNum), (POS, legNum), (POS, legNum)]
        '''
        self.client_futures = []
        for i in limbPositions:
            POS = i[0]
            legNum = i[1]
            request = SetJointAngles.Request()
            request.joint1 = POS[0]
            request.joint2 = POS[1]
            request.joint3 = -POS[2]
            self.client_futures.append(self.clients_[legNum - 1].call_async(request))
        self.spinFutures()

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


