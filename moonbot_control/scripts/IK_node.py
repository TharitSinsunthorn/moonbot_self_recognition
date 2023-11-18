#! /usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3

import numpy as np
from IK.limb_kinematics import InvKinematics

class IKNode(Node):

    def __init__(self):
        super().__init__("ik_node")

        self.RFpub = self.create_publisher(JointState, "RFstate", 10)

        self.plansub = self.create_subscription(Vector3, "set_rf_target", self.set_ik_callback, 10)

        # self.timer = self.create_timer(1, self.publish_joint_states)

        self.IK = InvKinematics()


    # def publish_joint_states(self):
    #     msg = JointState()
    #     msg.header.stamp = self.get_clock().now().to_msg()
    #     target = [1.0, 2.0, 3.0]
    #     angles = self.IK.get_joint_angles(target)
    #     angles = angles.tolist()
    #     msg.position = angles  # Replace with your joint positions
    #     self.RFpub.publish(msg)
    #     self.get_logger().info('Publishing joint states')

    def set_ik_callback(self, msg):
        # target = np.array([0.13, 0.0, 0.1], dtype=float)
        target = np.array([msg.x, msg.y, msg.z], dtype=float)
        angles = self.IK.get_joint_angles(target)
        # angles = angles.tolist()

        msg = JointState()
        msg.position = angles

        self.RFpub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    iknode = IKNode()
    executor = SingleThreadedExecutor()
    executor.add_node(iknode)
    try:
        executor.spin()
    except KeyboardInterrupt as e:
        iknode.get_logger().debug('KeyboardInterrupt caught, node shutting down cleanly\nbye bye <3')
    iknode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


