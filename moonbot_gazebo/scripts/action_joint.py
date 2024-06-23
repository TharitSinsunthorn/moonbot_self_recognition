#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from rclpy.duration import Duration
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
# from moonbot_custom_interfaces.msg import Detection

from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor


import numpy as np
import math
from IK.limb_kinematics import InvKinematics

import matplotlib.pyplot as plt


class JointPublisher(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('action_type', 'stand up'),
                ('use_sim', True)
            ]
        )

        self.action_type = self.get_parameter('action_type').get_parameter_value().string_value
        self.use_sim = self.get_parameter('use_sim').get_parameter_value().bool_value


        self.group = ReentrantCallbackGroup()

        ##### PUBLISHER ######
        if self.use_sim:
            self.RF_joint_publisher = self.create_publisher(
                JointTrajectory,
                'RFposition_trajectory_controller/joint_trajectory',
                10, 
                callback_group=self.group)

            self.LF_joint_publisher = self.create_publisher(
                JointTrajectory,
                'LFposition_trajectory_controller/joint_trajectory',
                10,
                callback_group=self.group)

            self.LR_joint_publisher = self.create_publisher(
                JointTrajectory,
                'LRposition_trajectory_controller/joint_trajectory',
                10,
                callback_group=self.group)

            self.RR_joint_publisher = self.create_publisher(
                JointTrajectory,
                'RRposition_trajectory_controller/joint_trajectory',
                10,
                callback_group=self.group)
            
        else:
            self.RF_joint_publisher = self.create_publisher(
                JointTrajectory,
                '/RF/position_trajectory_controller/joint_trajectory',
                10, 
                callback_group=self.group)

            self.LF_joint_publisher = self.create_publisher(
                JointTrajectory,
                '/LF/position_trajectory_controller/joint_trajectory',
                10,
                callback_group=self.group)

            self.LR_joint_publisher = self.create_publisher(
                JointTrajectory,
                '/LR/position_trajectory_controller/joint_trajectory',
                10,
                callback_group=self.group)

            self.RR_joint_publisher = self.create_publisher(
                JointTrajectory,
                '/RR/position_trajectory_controller/joint_trajectory',
                10,
                callback_group=self.group)
        ##### PUBLISHER #####

        self.IK = InvKinematics()

        ##### Pose Parameters #####
        self.span = 0.13 * np.sin(math.pi/4)
        self.height = 0.24

        ##### Pose Parameters #####

        self.ang_RF = []
        self.ang_LF = []
        self.ang_LR = []
        self.ang_RR = []
        self.all_joint_angles = []


    def start_fullconfig(self):

        h = 0.24
        span = 0.13
        self.intial_point_duration = 0.0
        self.sec = 0.95

        startconfig = self.IK.get_joint_angles([span, 0.0, -0.1])
        startconfig2 = self.IK.get_joint_angles([span, 0.0, h])

        self.ang_RF = [startconfig, startconfig, startconfig2]
        self.ang_LF = [startconfig, startconfig, startconfig2]
        self.ang_LR = [startconfig, startconfig, startconfig2] 
        self.ang_RR = [startconfig, startconfig, startconfig2]

    def sit_fullconfig(self):
        self.intial_point_duration = 0.0
        self.sec = 1
        sit = self.IK.get_joint_angles([0.13, 0.0, 0.0])

        self.ang_RF = [sit]
        self.ang_LF = [sit]
        self.ang_LR = [sit] 
        self.ang_RR = [sit]

    
    def pub_action(self):

        self.get_logger().info(f'{self.action_type}')

        if self.action_type == 'stand up':
            self.start_fullconfig()
        elif self.action_type == 'sit':
            self.sit_fullconfig()
        else:
            pass

        self.RobotPub()



    def RobotPub(self):
        RF_msg = JointTrajectory()
        LF_msg = JointTrajectory()
        LR_msg = JointTrajectory()
        RR_msg = JointTrajectory()

        joint_names_rf = ["j_c1_rf", "j_thigh_rf", "j_tibia_rf"]
        joint_names_lf = ["j_c1_lf", "j_thigh_lf", "j_tibia_lf"]
        joint_names_lr = ["j_c1_lr", "j_thigh_lr", "j_tibia_lr"]
        joint_names_rr = ["j_c1_rr", "j_thigh_rr", "j_tibia_rr"]

        RF = self.ang_RF 
        LF = self.ang_LF
        LR = self.ang_LR
        RR = self.ang_RR

        sec = self.sec

        RF_points = []
        LF_points = []
        LR_points = []
        RR_points = []
        for i in range(len(RF)):
            RF_point = JointTrajectoryPoint()
            LF_point = JointTrajectoryPoint()
            LR_point = JointTrajectoryPoint()
            RR_point = JointTrajectoryPoint()

            RF_point.time_from_start = Duration(seconds=(i+1)*sec + self.intial_point_duration, nanoseconds=0).to_msg()
            LF_point.time_from_start = Duration(seconds=(i+1)*sec + self.intial_point_duration, nanoseconds=0).to_msg()
            LR_point.time_from_start = Duration(seconds=(i+1)*sec + self.intial_point_duration, nanoseconds=0).to_msg()
            RR_point.time_from_start = Duration(seconds=(i+1)*sec + self.intial_point_duration, nanoseconds=0).to_msg()

            RF_point.positions = RF[i]
            LF_point.positions = LF[i]
            LR_point.positions = LR[i]
            RR_point.positions = RR[i]

            # point.velocities = vel[i]
            RF_points.append(RF_point)
            LF_points.append(LF_point)
            LR_points.append(LR_point)
            RR_points.append(RR_point)


        RF_msg.joint_names = joint_names_rf
        RF_msg.points = RF_points

        LF_msg.joint_names = joint_names_lf
        LF_msg.points = LF_points

        LR_msg.joint_names = joint_names_lr
        LR_msg.points = LR_points

        RR_msg.joint_names = joint_names_rr
        RR_msg.points = RR_points

        self.RF_joint_publisher.publish(RF_msg)
        self.LF_joint_publisher.publish(LF_msg)
        self.LR_joint_publisher.publish(LR_msg)
        self.RR_joint_publisher.publish(RR_msg)


def main(args=None):
    rclpy.init(args=args)

    joint_publisher = JointPublisher()

    joint_publisher.pub_action()

    # rclpy.spin(joint_publisher)

    joint_publisher.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
