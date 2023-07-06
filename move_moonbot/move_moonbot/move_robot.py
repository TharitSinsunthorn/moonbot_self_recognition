import keyboard
import rclpy
import math
from rclpy.node import Node
import time
from moonbot_custom_interfaces.srv import SetJointAngles


class MoveRobot(Node):
    def __init__(self):
        super().__init__('move_robot')
        self.clients_ = [self.create_client(SetJointAngles, f'set_joint_angles_l{i + 1}') for i in range(4)]
        for i in range(4):
            while not self.clients_[i].wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Service not available for Limb {i+1}, waiting ...')
        self.client_futures = []
        self.reset_position()
        self.set_initial_position()
        # self.reset_position()
        # self.moveX()
        # self.move_track = 0
        self.reset_position()
        # self.moveX()

    def spinFutures(self):
        for f in self.client_futures:
            rclpy.spin_until_future_complete(self, f)
        self.get_logger().info("Well I am done, so we are good")
        
    def reset_position(self):
        self.get_logger().info("Resetting position")
        HOME_POS = [-18.0, 49.12, -10.25]
        request = SetJointAngles.Request()
        request.joint1 = HOME_POS[0]
        request.joint2 = HOME_POS[1]
        request.joint3 = HOME_POS[2]
        for i in range(4):
            self.get_logger().info("Future request called")
            future = self.clients_[i].call_async(request)
            rclpy.spin_until_future_complete(self, future)

        
    
    def handleResponse(self, future):
        result = future.result()
        self.get_logger().info("One Limb Movement Finished")
        self.move_track += 1

    def set_initial_position(self):
        self.get_logger().info("Initialising position")
        self.move_track = 0
        HOME_POS = [1.0, 57.0, -75.0]
        request = SetJointAngles.Request()
        request.joint1 = HOME_POS[0]
        request.joint2 = HOME_POS[1]
        request.joint3 = HOME_POS[2]

        self.client_futures = []
        
        for i in range(4):
            future = self.client_futures.append(self.clients_[i].call_async(request))
        self.spinFutures()

    def moveX(self):
        self.get_logger().info("Moving in X direction")
        self.move_track = 0
        LIMB = [[-5.37512621234589, -34.51420420572208, 26.74643220773413], [-6.871807205705636, -77.32214594067739, 90.05261012270847],
                [9.108503819010082, -76.63268004125507, 89.22473482298955], [7.134760469268315, -33.24800661349525, 24.557008044400447]]
        for i in range(4):
            request = SetJointAngles.Request()
            request.joint1 = LIMB[i][0]
            request.joint2 = LIMB[i][1]
            request.joint3 = LIMB[i][2]
            future = self.clients_[i].call_async(request)
            future.add_done_callback(self.handleResponse)
        while(self.move_track != 4):
            time.sleep(0.5)

def main(args = None):
    rclpy.init(args = args)
    node1 = MoveRobot()
    rclpy.spin(node1)

    node1.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


