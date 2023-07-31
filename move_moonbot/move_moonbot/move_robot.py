import rclpy
from rclpy.node import Node
from moonbot_custom_interfaces.srv import SetJointAngles
from std_msgs.msg import String
from move_moonbot.utilities.saved_sequences import *

class MoveRobot(Node):
    def __init__(self):
        super().__init__('move_robot')
        self.get_logger().info("Move Robot Initialised.")
        self.clients_ = [self.create_client(SetJointAngles, f'set_joint_angles_l{i + 1}') for i in range(4)]
        for i in range(4):
            while not self.clients_[i].wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Service not available for Limb {i+1}, waiting ...')
        self.client_futures = []
        self.setPositions([(HOME_POS1, 1), (HOME_POS2, 2), (HOME_POS3, 3), (HOME_POS4, 4)])
        self.subscriberKey = self.create_subscription(String, 'cmd_bot', self.move_clbk, 1)

    def move_clbk(self, msg):
        if msg.data == 'up':
            self.moveXpos()
        elif msg.data == "down":
            self.moveXneg()
        elif msg.data == "left":
            self.moveYpos()
        elif msg.data == "right":
            self.moveYneg()

    def moveXpos(self):
        self.get_logger().info("Moving bot in x direction")
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

    def moveXneg(self):
        self.get_logger().info("Moving bot in -x direction")
        self.setPositions([(STAGE11, 3), (STAGE12, 4), (STAGE13, 1), (STAGE14, 2)]) 
        self.setPositions([(HOPI2, 4)])
        self.setPositions([(HOPM2, 4)])
        self.setPositions([(HOPF2, 4)])
        self.setPositions([(HOPI1, 3)])
        self.setPositions([(HOPM1, 3)])
        self.setPositions([(HOPF1, 3)])
        self.setPositions([(STAGE21, 3), (STAGE22, 4), (STAGE23, 1), (STAGE24, 2)])
        self.setPositions([(HOPI3, 1)])
        self.setPositions([(HOPM3, 1)])
        self.setPositions([(HOPF3, 1)])
        self.setPositions([(HOPI4, 2)])
        self.setPositions([(HOPM4, 2)])
        self.setPositions([(HOPF4, 2)])
        self.setPositions([(STAGE31, 3), (STAGE32, 4), (STAGE33, 1), (STAGE34, 2)])
    
    def moveYpos(self):
        self.get_logger().info("Moving bot in y direction")
        self.setPositions([(STAGE11, 4), (STAGE12, 1), (STAGE13, 2), (STAGE14, 3)]) 
        self.setPositions([(HOPI2, 1)])
        self.setPositions([(HOPM2, 1)])
        self.setPositions([(HOPF2, 1)])
        self.setPositions([(HOPI1, 4)])
        self.setPositions([(HOPM1, 4)])
        self.setPositions([(HOPF1, 4)])
        self.setPositions([(STAGE21, 4), (STAGE22, 1), (STAGE23, 2), (STAGE24, 3)])
        self.setPositions([(HOPI3, 2)])
        self.setPositions([(HOPM3, 2)])
        self.setPositions([(HOPF3, 2)])
        self.setPositions([(HOPI4, 3)])
        self.setPositions([(HOPM4, 3)])
        self.setPositions([(HOPF4, 3)])
        self.setPositions([(STAGE31, 4), (STAGE32, 1), (STAGE33, 2), (STAGE34, 3)])

    def moveYneg(self):
        self.get_logger().info("Moving bot in -y direction")
        self.setPositions([(STAGE11, 2), (STAGE12, 3), (STAGE13, 4), (STAGE14, 1)]) 
        self.setPositions([(HOPI2, 3)])
        self.setPositions([(HOPM2, 3)])
        self.setPositions([(HOPF2, 3)])
        self.setPositions([(HOPI1, 2)])
        self.setPositions([(HOPM1, 2)])
        self.setPositions([(HOPF1, 2)])
        self.setPositions([(STAGE21, 2), (STAGE22, 3), (STAGE23, 4), (STAGE24, 1)])
        self.setPositions([(HOPI3, 4)])
        self.setPositions([(HOPM3, 4)])
        self.setPositions([(HOPF3, 4)])
        self.setPositions([(HOPI4, 1)])
        self.setPositions([(HOPM4, 1)])
        self.setPositions([(HOPF4, 1)])
        self.setPositions([(STAGE31, 2), (STAGE32, 3), (STAGE33, 4), (STAGE34, 1)])

    
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
            request.joint3 = -POS[2] # the negative sign is because the servo angles are in opposite direction
            self.client_futures.append(self.clients_[legNum - 1].call_async(request))
        self.spinFutures()

def main(args = None):
    rclpy.init(args = args)
    node1 = MoveRobot()
    rclpy.spin(node1)

    node1.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


