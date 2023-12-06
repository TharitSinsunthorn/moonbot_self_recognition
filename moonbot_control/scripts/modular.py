#! /usr/bin/env python3
import time
import rclpy
import math
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.action import ActionClient

from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
from moonbot_custom_interfaces.msg import Detection
from control_msgs.action import FollowJointTrajectory


import numpy as np
from IK.limb_kinematics import InvKinematics



class JointPublisher(Node):

    def __init__(self):
        super().__init__('JointPublisher_Node')

        ##### PUBLISHER #####
        self.joint_publisher = self.create_publisher(
            JointTrajectory,
            '/position_trajectory_controller/joint_trajectory',
            10)
        ##### PUBLISHER #####

        ##### SUBSCRIBER #####
        self.LEGsub = self.create_subscription(Detection, 
            "moonbot_detection", 
            self.leg_callback, 
            10)
        self.LEGsub

        # self.LegState = self.create_subscription(JointTrajectoryControllerState,
        #     '/position_trajectory_controller/joint_trajectory',
        #   self.Leg,
        #   10)
        # self.legState
        ##### SUBSCRIBER #####

        ##### TIMER ######
        self.timer_period = 1  # seconds execute every 0.5 seconds
        self.timer = self.create_timer(self.timer_period, self.pub_callback)
        self.i = -10
        ##### TIMER ######

        self.IK = InvKinematics()
        self.pathrange = 40

        self.seq = []

        self.joint_names = ["j_c1_rf", "j_thigh_rf", "j_tibia_rf",
                            "j_c1_lf", "j_thigh_lf", "j_tibia_lf",
                            "j_c1_lr", "j_thigh_lr", "j_tibia_lr",
                            "j_c1_rr", "j_thigh_rr", "j_tibia_rr"]

        self.connected_joints = []


        self.ang_RF = []
        self.ang_LF = []
        self.ang_LR = []
        self.ang_RR = []
        self.all_joint_angles = []
        self.NoTar = [0, 0, 0]

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

        if self.NumCon == 1:
            self.SinPub()

        elif self.NumCon == 2:
            self.DuoPub()

        elif self.NumCon == 3:
            self.TriPub()

        elif self.NumCon == 4:
            self.QuadPub()

        elif self.NumCon == 0:
            self.connected_joints = []

        self.RobotPub(self.joint_names, self.seq)

        self.get_logger().info(f"Number of connection: {self.NumCon}")
        self.get_logger().info(f"Connection list: {self.connected_joints}")


    def SinPub(self):

        connected_port = self.generate_connected_port()
        self.generate_joints_name(connected_port)
                       
        ##### Single limb parameters #####      
        f = 0.08
        h = 0.24
        lift = 0.02
        span = 0.255
        self.repeat = 1
        ground = 0.05
        pathrange = self.pathrange
        sec = self.timer_period / pathrange / 2
        ##### Single limb parameters #####      

        # L = []
        # # plot = []
        # for j in range(pathrange):
        #     div = (f)/pathrange     
        #     x = span + div*j 
        #     z = -np.sqrt(lift**2 * (1 - (2*(x-span-f/2)/f)**2))
        #     # plot.append([x, 0.0, z + ground])
        #     L.append(self.IK.get_joint_angles([x, 0.0, z + ground]))
        # L.append(self.IK.get_joint_angles([span+f, 0.0, ground]))

        # for j in range(pathrange):
        #     div = (f)/pathrange     
        #     x = span + f - div*j 
        #     L.append(self.IK.get_joint_angles([x, 0.0, ground]))
        #     # plot.append([x, 0.0, ground])

        L = []
        # plot = []
        for j in range(pathrange):
            div = (f)/pathrange     
            x = div*j 
            z = -np.sqrt(lift**2 * (1 - (2*(x-f/2)/f)**2))
            # plot.append([x, 0.0, z + ground])
            L.append(self.IK.get_joint_angles([span, x, z + ground]))

        for j in range(pathrange):
            div = (f)/pathrange     
            x = f - div*j 
            L.append(self.IK.get_joint_angles([span, x, ground]))
            # plot.append([x, 0.0, ground])


        tar = L*self.repeat

        for i in range(len(tar)):
            self.seq.append(tar[0]+tar[0]+tar[0])
            # vel.append(vRF[i]+vRR[i]+vLR[i]+LF[i])
            for j in range(3):
                self.seq[i].insert(self.leg_connection_state.index(True)*3, tar[i][2-j])


    def DuoPub(self, all_joint_angles = None):\

        connected_port = self.generate_connected_port()
        self.generate_joints_name(connected_port)
                       
        ##### Single limb parameters #####      
        f = 0.08
        h = 0.24
        lift = 0.02
        span = 0.255
        self.repeat = 1
        ground = 0.05
        pathrange = self.pathrange
        sec = self.timer_period / pathrange / 2
        ##### Single limb parameters #####      

        L = []
        for j in range(pathrange):
            div = (f)/pathrange     
            x = span + div*j 
            z = -np.sqrt(lift**2 * (1 - (2*(x-span-f/2)/f)**2))
            # plot.append([x, 0.0, z + ground])
            L.append(self.IK.get_joint_angles([x, 0.0, z + ground]))
        L.append(self.IK.get_joint_angles([span+f, 0.0, ground]))

        for j in range(pathrange):
            div = (f)/pathrange     
            x = span + f - div*j 
            L.append(self.IK.get_joint_angles([x, 0.0, ground]))

        tar = L*self.repeat

        for i in range(len(tar)):
            self.seq.append(tar[0]+tar[0]+tar[0])
            # vel.append(vRF[i]+vRR[i]+vLR[i]+LF[i])
            for j in range(3):
                self.seq[i].insert(self.leg_connection_state.index(True)*3, tar[i][2-j])
                    
    


    def TriPub(self, all_joint_angles = None):

        connected_port = self.generate_connected_port()
        self.generate_joints_name(connected_port)


    def QuadPub(self, all_joint_angles = None):

        connected_port = self.generate_connected_port()
        self.generate_joints_name(connected_port)
                    
        sec = 0.1

        tar = self.IK.get_joint_angles([0.2, 0.0, 0.16])
        f = 0.05
        h = 0.24
        lift = 0.03
        span = 0.13
        self.repeat = 1

        tar1 = self.IK.get_joint_angles([span-f, -f, h])
        tar2 = self.IK.get_joint_angles([span+f/2, f/2, h-lift])
        tar3 = self.IK.get_joint_angles([span+f, f, h])
        tar4 = self.IK.get_joint_angles([span, 0.0, h])
        tar5 = self.IK.get_joint_angles([span, 0.0, h])

        tar6 = self.IK.get_joint_angles([span+f, f, h])
        tar7 = self.IK.get_joint_angles([span-f/2, -f/2, h-lift])
        tar8 = self.IK.get_joint_angles([span-f, -f, h])
        tar9 = self.IK.get_joint_angles([span, 0.0, h])
        tar10 = self.IK.get_joint_angles([span, 0.0, h])


        tar11 = self.IK.get_joint_angles([span, 0.0, h])
        tar12 = self.IK.get_joint_angles([span, 0.0, h])
        tar13 = self.IK.get_joint_angles([span-f, f, h])
        tar14 = self.IK.get_joint_angles([span+f/2, -f/2, h-lift])
        tar15 = self.IK.get_joint_angles([span+f, -f, h])


        tar16 = self.IK.get_joint_angles([span, 0.0, h])
        tar17 = self.IK.get_joint_angles([span, 0.0, h])
        tar18 = self.IK.get_joint_angles([span+f, -f, h])
        tar19 = self.IK.get_joint_angles([span-f/2, f/2, h-lift])
        tar20 = self.IK.get_joint_angles([span-f, f, h])


        RF = [tar1, tar2, tar3, tar4, tar5, tar5]*self.repeat
        LR = [tar6, tar7, tar8, tar9, tar10,tar10]*self.repeat
        LF = [tar11, tar11, tar12, tar13, tar14, tar15]*self.repeat
        RR = [tar16, tar16, tar17, tar18, tar19, tar20]*self.repeat
              

        RF.insert(0, RF[-1])
        RF.insert(0, RF[-1])

        LR.insert(0, LR[-1])
        LR.insert(0, LR[-1])

        LF.insert(0, LF[-1])
        LF.insert(0, LF[-2])
        LF[-2] = tar19
        LF[-1] = tar4

        RR.insert(0, RR[-1])
        RR.insert(0, RR[-2])
        RR[-2] = tar14
        RR[-1] = tar4

        # seq = []
        for i in range(len(LF)):
            self.seq.append(RF[i]+LF[i]+LR[i]+RR[i])
            # vel.append(vRF[i]+vRR[i]+vLR[i]+LF[i])


    def RobotPub(self, joint_names, seq):
        msg = JointTrajectory()
        sec = self.timer_period / self.pathrange / 2

        points = []
        for i in range(len(seq)):
            point = JointTrajectoryPoint()
            point.time_from_start = Duration(seconds=(i+1)*sec, nanoseconds=0).to_msg()
            point.positions = seq[i]
            # point.velocities = vel[i]
            points.append(point)


        msg.joint_names = joint_names
        msg.points = points
        # self.get_logger().info(f"{tar}")

        self.joint_publisher.publish(msg)
        self.seq.clear()

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

    rclpy.spin(joint_publisher)

    joint_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
