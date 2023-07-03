#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from moonbot_gazebo.src.moonbot import Moonbot


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('walker_demo')

    node.get_logger().info('Instantiating robot Client')
    robot = Moonbot()

    rclpy.sleep(node, 1)

    node.get_logger().info('Walker Demo Starting')

    robot.set_walk_velocity(0.2, 0, 0)
    rclpy.sleep(node, 3)
    robot.set_walk_velocity(1, 0, 0)
    rclpy.sleep(node, 3)
    robot.set_walk_velocity(0, 1, 0)
    rclpy.sleep(node, 3)
    robot.set_walk_velocity(0, -1, 0)
    rclpy.sleep(node, 3)
    robot.set_walk_velocity(-1, 0, 0)
    rclpy.sleep(node, 3)
    robot.set_walk_velocity(1, 1, 0)
    rclpy.sleep(node, 5)
    robot.set_walk_velocity(0, 0, 0)

    node.get_logger().info('Walker Demo Finished')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
