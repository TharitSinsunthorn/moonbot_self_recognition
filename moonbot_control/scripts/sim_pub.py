#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from rclpy.duration import Duration
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

import numpy as np
from IK.limb_kinematics import InvKinematics

class JointPublisher(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.joint_publisher = self.create_publisher(
            JointTrajectory,
            '/position_trajectory_controller/joint_trajectory',
            10)

        # self.RFsub = self.create_subscription(JointState, 
        #     "RFstate", 
        #     self.RFcallback, 10)
        # self.RFsub

        self.IK = InvKinematics()

        self.tar = []

        self.target_received = False


    def RFcallback(self, msg):
        # self.mis = msg.position
        self.get_logger().info(f'Received joint states: {msg.position}')
        self.tar = msg.position
        self.tar = self.tar.tolist()
        self.target_received = True
        # print(self.tar)


    def target_checker(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.target_received:
                self.pubpub()
                # break
            # self.target_received = False
            # rclpy.spin_once(self)
            

    def pubpub(self):
        msg = JointTrajectory()

        joint_names = ["j_c1_rf", "j_thigh_rf", "j_tibia_rf",
                       "j_c1_rr", "j_thigh_rr", "j_tibia_rr",
                       "j_c1_lr", "j_thigh_lr", "j_tibia_lr",
                       "j_c1_lf", "j_thigh_lf", "j_tibia_lf"]

        sec = 2.0

        f = -0.06
        h = 0.24
        tar0 = self.IK.get_joint_angles([0.13, 0.0, -0.1])
        tar1 = self.IK.get_joint_angles([0.13, 0.0, h])
        sit = self.IK.get_joint_angles([0.13, 0.0, 0.1])

        test = self.IK.get_joint_angles(np.array([0.13, 0.0, h]))
        # print(self.tar)

        # standup seq
        RF = [tar0,tar0,tar1]
        RR = [tar0,tar0,tar1]
        LR = [tar0,tar0,tar1]
        LF = [tar0,tar0,tar1]

        seq = []
        for i in range(len(LF)):
            seq.append(RF[i]+RR[i]+LR[i]+LF[i])
            # vel.append(vRF[i]+vRR[i]+vLR[i]+LF[i])
        # seq = seq*6
        # print(seq)

        points = []
        for i in range(len(seq)):
            point = JointTrajectoryPoint()
            point.time_from_start = Duration(seconds=(i+1)*sec, nanoseconds=0).to_msg()
            point.positions = seq[i]
            # point.velocities = vel[i]
            points.append(point)
            # print(points)


        msg.joint_names = joint_names
        msg.points = points

        self.joint_publisher.publish(msg)

        


def main(args=None):
    rclpy.init(args=args)

    joint_publisher = JointPublisher()

    # joint_publisher.target_checker()
    joint_publisher.pubpub()

    # rclpy.spin(joint_publisher)

    joint_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()