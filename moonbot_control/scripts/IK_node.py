#! /usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor


from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
from moonbot_custom_interfaces.msg import SetPosition, Geometry

import numpy as np
from IK.limb_kinematics import InvKinematics

from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor


class JointPublisher(Node):

    def __init__(self):
        super().__init__('JointPublisher_Node')

        self.group = ReentrantCallbackGroup()

        ##### PUBLISHER #####
        self.joint_publisher = self.create_publisher(
            JointTrajectory,
            '/position_trajectory_controller/joint_trajectory',
            10)

        self.RF_joint_publisher = self.create_publisher(
            JointTrajectory,
            '/RF/position_trajectory_controller/joint_trajectory',
            1, 
            callback_group=self.group)

        self.LF_joint_publisher = self.create_publisher(
            JointTrajectory,
            '/LF/position_trajectory_controller/joint_trajectory',
            1,
            callback_group=self.group)

        self.LR_joint_publisher = self.create_publisher(
            JointTrajectory,
            '/LR/position_trajectory_controller/joint_trajectory',
            1,
            callback_group=self.group)

        self.RR_joint_publisher = self.create_publisher(
            JointTrajectory,
            '/RR/position_trajectory_controller/joint_trajectory',
            1,
            callback_group=self.group)
        ##### PUBLISHER #####

        ##### SUBSCRIBER
        self.LEGsub = self.create_subscription(Geometry, 
            "moonbot_geometry", 
            self.LEGcallback, 
            10)
        self.LEGsub
        ##### SUBSCRIBER #####

        ##### TIMER ######
        self.timer_period = 0.1 # seconds execute every 0.5 seconds
        self.timer = self.create_timer(self.timer_period, self.pub_callback)
        # self.i = -10
        ##### TIMER ######

        self.IK = InvKinematics()

        self.tar = []

        self.target_received = False

        self.ang_RF = []
        self.ang_LF = []
        self.ang_LR = []
        self.ang_RR = []
        self.all_joint_angles = []
        self.prev_joint_angs = None


    def pub_callback(self):
        # print(self.IK.get_RF_joint_angles([0.2, 0.0, 0.0], [0.05, 0, 0.0]))
        # print(self.IK.get_LF_joint_angles([0.2, 0.0, 0.0], [0.05, 0, 0.0]))
        # print(self.IK.get_LR_joint_angles([0.2, 0.0, 0.0], [0.05, 0, 0.0]))
        # print(self.IK.get_RR_joint_angles([0.2, 0.0, 0.0], [0.05, 0, 0.0]))
        if self.all_joint_angles != None:
            # print(self.all_joint_angles)
            self.pubpub(self.all_joint_angles)
          


    def LEGcallback(self, msg):
        # self.mis = msg.position
        # self.get_logger().info(f'Received joint states:')

        eulerAng = np.array([msg.euler_ang.x, msg.euler_ang.y, msg.euler_ang.z])
        rf_coord = np.array([msg.rf.x, msg.rf.y, msg.rf.z])
        lf_coord = np.array([msg.lf.x, msg.lf.y, msg.lf.z])
        lr_coord = np.array([msg.lr.x, msg.lr.y, msg.lr.z])
        rr_coord = np.array([msg.rr.x, msg.rr.y, msg.rr.z])

        # eulerAng = np.array([0,0,0])
        # rf_coord = np.array([0.13, 0.0, msg.rf.z])
        # lf_coord = np.array([0.13, 0.0, msg.lf.z])
        # lr_coord = np.array([0.13, 0.0, msg.lr.z])
        # rr_coord = np.array([0.13, 0.0, msg.rr.z])

        self.ang_RF = self.IK.get_RF_joint_angles(rf_coord, eulerAng)
        self.ang_LF = self.IK.get_LF_joint_angles(lf_coord, eulerAng)
        self.ang_LR = self.IK.get_LR_joint_angles(lr_coord, eulerAng)
        self.ang_RR = self.IK.get_RR_joint_angles(rr_coord, eulerAng)

        # print(f'RF: {self.ang_RF}')
        # print(f'LF: {self.ang_LF}')
        # print(f'LR: {self.ang_LR}')
        # print(f'RR: {self.ang_RR}')
  
        
        self.prev_joint_angs = self.all_joint_angles

        # self.get_logger().info(f'all: {self.all_joint_angles}')

        self.target_received = True


    def target_checker(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.target_received:
                self.pubpub()
                # break
            # self.target_received = False
            # rclpy.spin_once(self)
            

    def pubpub(self, all_joint_angles):
        # msg = JointTrajectory()

        RF_msg = JointTrajectory()
        LF_msg = JointTrajectory()
        LR_msg = JointTrajectory()
        RR_msg = JointTrajectory()


        joint_names_rf = ["j_c1_rf", "j_thigh_rf", "j_tibia_rf"]
        joint_names_lf = ["j_c1_lf", "j_thigh_lf", "j_tibia_lf"]
        joint_names_lr = ["j_c1_lr", "j_thigh_lr", "j_tibia_lr"]
        joint_names_rr = ["j_c1_rr", "j_thigh_rr", "j_tibia_rr"]
                       
                    
        sec = self.timer_period

        RF = [self.ang_RF] 
        LF = [self.ang_LF]
        LR = [self.ang_LR]
        RR = [self.ang_RR]


        RF_points = []
        LF_points = []
        LR_points = []
        RR_points = []
        for i in range(len(RF)):
            RF_point = JointTrajectoryPoint()
            LF_point = JointTrajectoryPoint()
            LR_point = JointTrajectoryPoint()
            RR_point = JointTrajectoryPoint()

            RF_point.time_from_start = Duration(seconds=(i+1)*sec , nanoseconds=0).to_msg()
            LF_point.time_from_start = Duration(seconds=(i+1)*sec , nanoseconds=0).to_msg()
            LR_point.time_from_start = Duration(seconds=(i+1)*sec , nanoseconds=0).to_msg()
            RR_point.time_from_start = Duration(seconds=(i+1)*sec , nanoseconds=0).to_msg()

            RF_point.positions = RF[i]
            LF_point.positions = LF[i]
            LR_point.positions = LR[i]
            RR_point.positions = RR[i]

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

    rclpy.spin(joint_publisher)

    joint_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()


