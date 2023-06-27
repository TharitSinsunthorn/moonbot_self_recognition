import rclpy
import math
from rclpy.node import Node

from geometry_msgs.msg import Point

'''
This Node performs the gait analysis and publishes the limb tip position for all the limbs
'''
class BodyController(Node):
    def __init__(self):
        super().__init__('body_controller')
        self.publisher_angles = [self.create_publisher(Point, f'target_tip_position_l{i+1}', 10) for i in range(4)]

def main(args = None):
    rclpy.init(args = args)
    node1 = BodyController()
    rclpy.spin(node1)

    node1.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


