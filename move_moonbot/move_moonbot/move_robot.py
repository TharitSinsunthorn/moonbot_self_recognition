import keyboard
import rclpy
import math
from rclpy.node import Node
import time
from moonbot_custom_interfaces.srv import SetJointAngles


# Note that all joint3 angles are published in opposite angles. This is because of some problem with servo connection or idk.

HOME_POS1 = [0.0, 5.0, -115.0]
HOME_POS2 = [0.0, 5.0, -115.0]
HOME_POS3 = [0.0, 5.0, -115.0]
HOME_POS4 = [0.0, 5.0, -115.0]
STAGE11 = [33.81266267289558, -8.447375495254537, -121.36386684745158]
STAGE12 = [15.975425232549906, 8.725731805269717, -89.4766988058537]
STAGE13 = [-15.975425232549906, 8.725731805269717, -89.4766988058537]
STAGE14 = [-33.81266267289558, -8.447375495254537, -121.36386684745158]
HOPI2 = [15.975425232549906, 60.988270322240737, -105.48972974367456]
HOPM2 = [-76.15080729501362, 22.079952222681893, -140.75753391433332]
HOPF2 = [-76.15080729501362, -2.3240959163947394, -119.88607188066825]
HOPI1 = [33.81266267289558, 60.032633807984126, -142.81328526152583]
HOPM1 = [-23.99528663381575, 7.362509760753943, -51.45042615744111]
HOPF1 = [-23.99528663381575, -16.22542337035274, -25.921949231986975]
STAGE21 = [-15.975425232549888, 8.725731805269717, -89.4766988058537]
STAGE22 = [-33.81266267289556, -8.447375495254537, -121.36386684745158]
STAGE23 = [-23.995286633815756, -16.22542337035274, -25.921949231986975]
STAGE24 = [-76.15080729501364, -2.3240959163947394, -119.88607188066825]
HOPI3 = [-23.995286633815756, 60.362509760753943, -51.45042615744111]
HOPM3 = [33.81266267289558, 14.032633807984112, -142.81328526152583]
HOPF3 = [33.81266267289558, -8.44737549525455, -121.36386684745158]
HOPI4 = [-76.15080729501364, 60.079952222681893, -140.75753391433332]
HOPM4= [15.975425232549888, 29.988270322240737, -105.48972974367456]
HOPF4 = [15.975425232549888, 8.725731805269717, -89.4766988058537]
STAGE31 = [0.0, 4.999999999999995, -114.99999999999997]
STAGE32 = [5.011655339005834e-15, 4.999999999999995, -115.0]
STAGE33 = [0.0, 5.000000000000007, -115.00000000000003]
STAGE34 = [0.0, 4.999999999999995, -114.99999999999997]

class MoveRobot(Node):
    def __init__(self):
        super().__init__('move_robot')
        self.clients_ = [self.create_client(SetJointAngles, f'set_joint_angles_l{i + 1}') for i in range(4)]
        for i in range(4):
            while not self.clients_[i].wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Service not available for Limb {i+1}, waiting ...')
        self.client_futures = []
        self.setPositions([(HOME_POS1, 1), (HOME_POS2, 2), (HOME_POS3, 3), (HOME_POS4, 4)])
        self.setPositions([(STAGE11, 1), (STAGE12, 2), (STAGE13, 3), (STAGE14, 4)]) 
        self.setPositions([(HOPI2, 2)])
        self.setPositions([(HOPM2, 2)])
        self.setPositions([(HOPF2, 2)])
        self.setPositions([(HOPI1, 1)])
        self.setPositions([(HOPM1, 1)])
        self.setPositions([(HOPF1, 1)])
        self.setPositions([(STAGE21, 1), (STAGE22, 2), (STAGE23, 3), (STAGE24, 4)])
        self.setPositions([(HOPI3, 3)])
        self.setPositions([(HOPM3, 3)])
        self.setPositions([(HOPF3, 3)])
        self.setPositions([(HOPI4, 4)])
        self.setPositions([(HOPM4, 4)])
        self.setPositions([(HOPF4, 4)])
        # self.setPositions([(STAGE31, 1), (STAGE32, 2), (STAGE33, 3), (STAGE34, 4)])

        self.setPositions([(STAGE11, 1), (STAGE12, 2), (STAGE13, 3), (STAGE14, 4)]) 
        self.setPositions([(HOPI2, 2)])
        self.setPositions([(HOPM2, 2)])
        self.setPositions([(HOPF2, 2)])
        self.setPositions([(HOPI1, 1)])
        self.setPositions([(HOPM1, 1)])
        self.setPositions([(HOPF1, 1)])
        self.setPositions([(STAGE21, 1), (STAGE22, 2), (STAGE23, 3), (STAGE24, 4)])
        self.setPositions([(HOPI3, 3)])
        self.setPositions([(HOPM3, 3)])
        self.setPositions([(HOPF3, 3)])
        self.setPositions([(HOPI4, 4)])
        self.setPositions([(HOPM4, 4)])
        self.setPositions([(HOPF4, 4)])
        self.setPositions([(STAGE31, 1), (STAGE32, 2), (STAGE33, 3), (STAGE34, 4)])

        # self.setPositions([(STAGE11, 1), (STAGE12, 2), (STAGE13, 3), (STAGE14, 4)]) 
        # self.setPositions([(HOPI2, 2)])
        # self.setPositions([(HOPM2, 2)])
        # self.setPositions([(HOPF2, 2)])
        # self.setPositions([(HOPI1, 1)])
        # self.setPositions([(HOPM1, 1)])
        # self.setPositions([(HOPF1, 1)])
        # self.setPositions([(STAGE21, 1), (STAGE22, 2), (STAGE23, 3), (STAGE24, 4)])
        # self.setPositions([(HOPI3, 3)])
        # self.setPositions([(HOPM3, 3)])
        # self.setPositions([(HOPF3, 3)])
        # self.setPositions([(HOPI4, 4)])
        # self.setPositions([(HOPM4, 4)])
        # self.setPositions([(HOPF4, 4)])
        # self.setPositions([(STAGE31, 1), (STAGE32, 2), (STAGE33, 3), (STAGE34, 4)])

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


