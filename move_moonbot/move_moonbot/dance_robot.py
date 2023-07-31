import keyboard
import rclpy
import math
from rclpy.node import Node
import time
from moonbot_custom_interfaces.srv import SetJointAngles


# Note that all joint3 angles are published in opposite angles. This is because of some problem with servo connection or idk.

HOME_POS3 = [0.0, -45.0, -45.0]
HOME_POS4 = [0.0, -45.0, -45.0]
HOME_POS1 = [0.0, -45.0, -45.0]
HOME_POS2 = [0.0, -45.0, -45.0]
STABLE3 = [-24.2386672162116, -45.8837962702258, -36.895027687569026]
STABLE4 = [0.0, -57.01472234956069, -58.36628600432607]
STABLE1 = [24.2386672162116, -45.8837962702258, -36.895027687569026]
STABLE2 = [0.0, 90.0, -90.0]

STEP13 = [-19.359391552986747, -16.475410413958905, -58.9702406689333]
STEP14 = [-27.105068173994294, -29.74328263891636, -95.94867577636246]
STEP11 = [32.06841096582305, -19.298077641686646, -91.70626762343771]
STEP12 = [0.0, 90.0, -90.0]

STEP23 = [-32.06841096582305, -19.298077641686646, -91.70626762343771]
STEP24 = [27.105068173994294, -29.74328263891636, -95.94867577636246]
STEP21 = [19.359391552986747, -16.475410413958905, -58.9702406689333]
STEP22 = [0.0, 90.0, -90.0]

STEP33 = [-24.2386672162116, -3.392014341772335, -92.10107930164428]
STEP34 = [0.0, -25.531073230138624, -108.17451282597514]
STEP31 = [24.2386672162116, -3.392014341772335, -92.10107930164428]
STEP32 = [0.0, 90.0, 120.0]


class DanceRobot(Node):
    def __init__(self):
        super().__init__('move_robot')
        self.clients_ = [self.create_client(SetJointAngles, f'set_joint_angles_l{i + 1}') for i in range(4)]
        for i in range(4):
            while not self.clients_[i].wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Service not available for Limb {i+1}, waiting ...')
        self.client_futures = []
        self.get_logger().info("Initialising the Robot")
        # self.setPositions([(HOME_POS1, 1), (HOME_POS2, 2), (HOME_POS3, 3), (HOME_POS4, 4)])
        # self.setPositions([(STABLE1, 1), (STABLE2, 2), (STABLE3, 3), (STABLE4, 4)])
        # self.sit()
        self.snakeMotion()

    
    def crawl(self):
        STARTPOS = [0.0, 64.0, -130.0]
        ENDPOS3 = [-35.7555320395302, 49.504298141959495, -103.81178385617089]
        ENDPOS4 = [35.7555320395302, 49.504298141959495, -103.81178385617089]
        ENDRESET = [0.0, 90.0, -90.0]
        
        while True:
            # Movement
            self.setPositions([(ENDRESET, 4)])
            self.setPositions([(STARTPOS, 4)])
            self.setPositions([(ENDRESET, 3)])
            self.setPositions([(STARTPOS, 3)])
            self.setPositions([(ENDPOS3, 3), (ENDPOS4, 4)])


    def muscleShowOff(self):
        self.get_logger().info("See my muscles?")
        STARTPOS = [0.0, 90.0, 30.0]
        ENDPOS = [-10.0, 20.0, 110.0]
        # STARTPOS = [0.0, 30.0, 0.0]
        # ENDPOS = [0.0, 90.0, 45.0]
        # while True:
        self.setPositions([(STARTPOS, 3), (STARTPOS, 4)])
        self.setPositions([(ENDPOS, 3), (ENDPOS, 4)])
    def snakeMotion(self):
        self.get_logger().info("Snake movement")
        STARTPOS = [0.0, 64.0, -130.0]
        ENDPOS = [0.0, 38.0, -84.0]
        ENDRESET = [0.0, 90.0, -90.0]
        self.setPositions([(ENDRESET, 4)])
        self.setPositions([(STARTPOS, 4)])
        self.setPositions([(ENDRESET, 3)])
        self.setPositions([(ENDPOS, 3)])

        while True:
            # Movement
            self.setPositions([(ENDPOS, 4), (STARTPOS, 3)])
            self.setPositions([(ENDRESET, 3)])
            self.setPositions([(ENDPOS, 3)])
            self.setPositions([(ENDRESET, 4)])
            self.setPositions([(STARTPOS, 4)])


        




    def step1(self):
        self.get_logger().info("Step 1")
        self.setPositions([(STEP11, 1), (STEP12, 2), (STEP13, 3), (STEP14, 4)])
        self.setPositions([(STABLE1, 1), (STABLE2, 2), (STABLE3, 3), (STABLE4, 4)])
    
    def step2(self):
        self.get_logger().info("Step 2")
        self.setPositions([(STEP21, 1), (STEP22, 2), (STEP23, 3), (STEP24, 4)])
        self.setPositions([(STABLE1, 1), (STABLE2, 2), (STABLE3, 3), (STABLE4, 4)])

    def sit(self):
        self.get_logger().info("Step 3")
        self.setPositions([(STEP31, 1), (STEP32, 2), (STEP33, 3), (STEP34, 4)])
        # self.setPositions([(STABLE1, 1), (STABLE2, 2), (STABLE3, 3), (STABLE4, 4)])
        
    def dance(self):
        self.get_logger().info("Dancing")
        self.step1()
        self.step2()
        self.step1()
        self.step2()
        self.step1()
        self.step2()
        self.step1()
        self.step2()
        self.sit()


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

def main(args = None):
    rclpy.init(args = args)
    node1 = DanceRobot()
    rclpy.spin(node1)

    node1.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


