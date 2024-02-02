#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from rclpy.duration import Duration
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point

from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
import time
import numpy as np
from IK.limb_kinematics import InvKinematics

class JointPublisher(Node):

    def __init__(self):
        super().__init__('Fowardkinematics_Node')

        self.group = ReentrantCallbackGroup()

        ##### SUBSCRIBER #####
        self.joint_state = self.create_subscription(
            JointState,
            '/joint_states',
            self.EE_callback,
            10)
        ##### SUBSCRIBER #####

        ##### PUBLISHER #####
        self.RF_EE_pose = self.create_publisher(
            Point,
            'RF_pose',
            10
            )

        self.LF_EE_pose = self.create_publisher(
            Point,
            'LF_pose',
            10
            )

        # self.LR_EE_pose = self.create_publisher(
        #     Point,
        #     'LR_pose',
        #     10
        #     )

        # self.RF_EE_pose = self.create_publisher(
        #     Point,
        #     'RR_pose',
        #     10
        #     )
        ##### PUBLISHER #####

        ##### TIMER ######
        self.timer_period = 2 
        self.timer = self.create_timer(self.timer_period, self.forward_callback)
        ##### TIMER ######

        self.IK = InvKinematics()

        self.pointRF = []


    def EE_callback(self, state):
        pointRF = Point()
        pointLF = Point()

        poseRF = self.IK.get_EE_position([state.position[0], state.position[4], state.position[8]])
        poseLF = self.IK.get_EE_position([state.position[1], state.position[5], state.position[9]])

        pointRF.x = poseRF[0]
        pointRF.y = poseRF[1]
        pointRF.z = poseRF[2]

        pointLF.x = poseLF[0]
        pointLF.y = poseLF[1]
        pointLF.z = poseLF[2]
        
        self.RF_EE_pose.publish(pointRF)
        self.LF_EE_pose.publish(pointLF)

        self.pointRF = [poseRF[0], poseRF[1], poseRF[2]]

    def forward_callback(self):
        self.get_logger().info(f"{self.pointRF}")


def main(args=None):
    rclpy.init(args=args)

    joint_publisher = JointPublisher()

    # joint_publisher.target_checker()
    # joint_publisher.pubpub()

    rclpy.spin(joint_publisher)

    joint_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

