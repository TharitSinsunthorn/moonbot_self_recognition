#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from rclpy.duration import Duration
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from moonbot_custom_interfaces.msg import Detection

from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor


import numpy as np
import math
from IK.limb_kinematics import InvKinematics

class JointPublisher(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

        self.group = ReentrantCallbackGroup()

        ##### PUBLISHER ######
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
        ##### PUBLISHER #####

        ##### SUBSCRIBER #####
        self.LEGsub = self.create_subscription(Detection, 
            "moonbot_detection", 
            self.leg_callback, 
            10)
        self.LEGsub
        ##### SUBSCRIBER #####

        ##### TIMER ######
        self.timer_period = 1  # seconds execute every 0.5 seconds
        self.timer = self.create_timer(self.timer_period + 1, self.pub_callback)
        ##### TIMER ######

        self.IK = InvKinematics()

        self.pathrange = 20

        self.tar = []

        self.joint_names = ["j_c1_rf", "j_thigh_rf", "j_tibia_rf",
                            "j_c1_lf", "j_thigh_lf", "j_tibia_lf",
                            "j_c1_lr", "j_thigh_lr", "j_tibia_lr",
                            "j_c1_rr", "j_thigh_rr", "j_tibia_rr"]

        self.ang_RF = []
        self.ang_LF = []
        self.ang_LR = []
        self.ang_RR = []
        self.all_joint_angles = []

        self.leg_connection_state = [False, False, False, False]
        self.leg_connection_state_prev = [False, False, False, False]

        self.NumCon = 0
        self.NumCon_prev = 0
        self.StateChanged = False


    def leg_callback(self, msg):
        
        for i in range(4):
            self.leg_connection_state[i] = msg.data[i]

        self.NumCon = sum(self.leg_connection_state)

        if self.NumCon > self.NumCon_prev:
            self.get_logger().info("Leg is ADDED")
            self.NumCon_prev = self.NumCon
            self.StateChanged = True

        elif self.NumCon < self.NumCon_prev:
            self.get_logger().info("Leg is LOSE")
            self.NumCon_prev = self.NumCon
            self.StateChanged = True 
        else:
            self.StateChanged = False


    def pub_callback(self):

        if self.StateChanged:
            # self.timer.cancel()
            self.get_logger().info('CHANGED!')

        if self.NumCon != 0: 

            if self.NumCon == 1:
                self.SinPub()

            elif self.NumCon == 2:
                self.DuoPub()

            elif self.NumCon == 3:
                self.TriPub()

            elif self.NumCon == 4:
                self.QuadPub()

            self.RobotPub(self.tar)

        elif self.NumCon == 0:
            self.connected_joints = []

        

        self.get_logger().info(f"Number of connection: {self.NumCon}")
        self.get_logger().info(f"Connection list: {self.connected_joints}")


    def SinPub(self):

        connected_port = self.generate_connected_port()
        self.generate_joints_name(connected_port)
                       
        ##### Single limb parameters #####      
        f = 0.14
        lift = 0.06
        span = 0.2
        self.repeat = 1
        ground = 0.05
        pathrange = self.pathrange
        sec = self.timer_period / pathrange / 2
        ##### Single limb parameters #####      

        traj = []
        plot = []
        for j in range(1, pathrange):
            div = (f)/pathrange     
            x = span + div*j 
            z = -np.sqrt(lift**2 * (1 - (2*(x-span-f/2)/f)**2))
            plot.append([x, 0.0, z + ground])
            traj.append(self.IK.get_joint_angles([x, 0.0, z + ground]))

        for j in range(1,pathrange):
            div = (f)/pathrange     
            x = span + f - div*j 
            traj.append(self.IK.get_joint_angles([x, 0.0, ground]))
            plot.append([x, 0.0, ground])
        # traj.append(self.IK.get_joint_angles([span, 0.0, ground]))

        self.ang_RF = traj
        self.ang_LF = traj
        self.ang_LR = traj
        self.ang_RR = traj


    def DuoPub(self):

        connected_port = self.generate_connected_port()

        self.generate_joints_name(connected_port)
                       
        ##### Single limb parameters #####      
        f = 0.12
        ff = 0.1
        h = 0.22
        lift = 0.2
        span = 0.22
        self.repeat = 1
        ground = 0.15
        pathrange = self.pathrange
        sec = self.timer_period / pathrange / 2
        ##### Single limb parameters ##### 

        
        trajDummy = []
        trajL = []
        trajR = []
        plot = []

        config_type = abs(connected_port[1]-connected_port[0])
        self.get_logger().info(f"connected: {connected_port}")

        if config_type == 2:

            for j in range(1, pathrange):
                div = (ff)/pathrange     
                x = div*j 
                z = -np.sqrt(lift**2 * (1 - (2*(x-ff/2)/ff)**2))
                # plot.append([x, 0.0, z + ground])
                trajDummy.append([0.0, 0.0, 0.0])
                trajL.append(self.IK.get_joint_angles([span, -x, z + ground]))
                trajR.append(self.IK.get_joint_angles([span, x, z + ground]))

            for j in range(1,pathrange):
                div = (ff)/pathrange     
                x = ff - div*j
                trajDummy.append([0.0, 0.0, 0.0])
                trajL.append(self.IK.get_joint_angles([span, -x, z + ground]))
                trajR.append(self.IK.get_joint_angles([span, x, z + ground]))
                # plot.append([x, 0.0, ground]) 

            if connected_port[0] == 0:
                self.ang_RF = trajR
                self.ang_LF = trajDummy
                self.ang_LR = trajL
                self.ang_RR = trajDummy

            elif connected_port[0] == 1:
                self.ang_RF = trajDummy
                self.ang_LF = trajL
                self.ang_LR = trajDummy
                self.ang_RR = trajR
            
        elif config_type == 1 or config_type == 3:

            for j in range(1, pathrange):
                div = (f)/pathrange     
                x = span + div*j 
                z = -np.sqrt(lift**2 * (1 - (2*(x-span-f/2)/f)**2))
                # plot.append([x, 0.0, z + ground])
                trajDummy.append([0.0, 0.0, 0.0])
                trajL.append(self.IK.get_joint_angles([x, 0.0, z + ground], [0, 0, -math.pi/4]))
                trajR.append(self.IK.get_joint_angles([x, 0.0, z + ground], [0, 0, math.pi/4]))

            for j in range(1,pathrange):
                div = (f)/pathrange     
                x = span + f - div*j
                trajDummy.append([0.0, 0.0, 0.0])
                trajL.append(self.IK.get_joint_angles([x, 0.0, z + ground], [0, 0, -math.pi/4]))
                trajR.append(self.IK.get_joint_angles([x, 0.0, z + ground], [0, 0, math.pi/4]))
                # plot.append([x, 0.0, ground])

            if connected_port[0] == 0 and connected_port[1] != 3:
                self.ang_RF = trajL
                self.ang_LF = trajR
                self.ang_LR = trajDummy
                self.ang_RR = trajDummy

            elif connected_port[0] == 1: 
                self.ang_RF = trajDummy
                self.ang_LF = trajL
                self.ang_LR = trajR
                self.ang_RR = trajDummy

            elif connected_port[0] == 2:
                self.ang_RF = trajDummy
                self.ang_LF = trajDummy
                self.ang_LR = trajL
                self.ang_RR = trajR

            elif connected_port[0] == 0 and connected_port[1] == 3:
                self.ang_RF = trajR
                self.ang_LF = trajDummy
                self.ang_LR = trajDummy
                self.ang_RR = trajL
            
            
    def TriPub(self):
        pass

    def QuadPub(self):
        pass

 
    def RobotPub(self, seq):
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

        sec = self.timer_period / len(RF)

        RF_points = []
        LF_points = []
        LR_points = []
        RR_points = []
        for i in range(len(RF)):
            RF_point = JointTrajectoryPoint()
            LF_point = JointTrajectoryPoint()
            LR_point = JointTrajectoryPoint()
            RR_point = JointTrajectoryPoint()

            RF_point.time_from_start = Duration(seconds=(i)*sec + 1, nanoseconds=0).to_msg()
            LF_point.time_from_start = Duration(seconds=(i)*sec + 1, nanoseconds=0).to_msg()
            LR_point.time_from_start = Duration(seconds=(i)*sec + 1, nanoseconds=0).to_msg()
            RR_point.time_from_start = Duration(seconds=(i)*sec + 1, nanoseconds=0).to_msg()

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


    def generate_connected_port(self):
        return [i for i, value in enumerate(self.leg_connection_state) if value]


    def generate_joints_name(self, connected_port):
        connected_joints = []
        for port in connected_port:
            for j in range(3):
                connected_joints.append(self.joint_names[port*3 + j])
        self.connected_joints = connected_joints


def main(args=None):
    rclpy.init(args=args)

    joint_publisher = JointPublisher()

    # joint_publisher.target_checker()
    # rclpy.spin(joint_publisher)
    # joint_publisher.pubpub()

    rclpy.spin(joint_publisher)

    joint_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

