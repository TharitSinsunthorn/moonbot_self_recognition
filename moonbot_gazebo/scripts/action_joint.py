#! /usr/bin/env python3
import time
import rclpy
import math
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor


from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
from moonbot_custom_interfaces.msg import SetPosition

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

        ##### SUBSCRIBER
        self.LEGsub = self.create_subscription(SetPosition, 
            "Moonbot_geometry", 
            self.LEGcallback, 
            10)
        self.LEGsub
        ##### SUBSCRIBER #####

        ##### TIMER ######
        self.timer_period = 1  # seconds execute every 0.5 seconds
        self.timer = self.create_timer(self.timer_period, self.pub_callback)
        self.i = -10
        ##### TIMER ######

        self.IK = InvKinematics()

        self.tar = []

        self.target_received = False

        self.ang_RF = []
        self.ang_LF = []
        self.ang_LR = []
        self.ang_RR = []
        self.all_joint_angles = []


    def pub_callback(self):
        print(self.IK.get_RF_joint_angles([0.2, 0.0, 0.0], [0.05, 0, 0.0]))
        print(self.IK.get_LF_joint_angles([0.2, 0.0, 0.0], [0.05, 0, 0.0]))
        print(self.IK.get_LR_joint_angles([0.2, 0.0, 0.0], [0.05, 0, 0.0]))
        print(self.IK.get_RR_joint_angles([0.2, 0.0, 0.0], [0.05, 0, 0.0]))
        if self.all_joint_angles != None:
            A = [0.0, 0.1, -0.1]
            B = [0.13, 0.0, 0.24]
            self.pubpub([self.IK.get_RF_joint_angles(B, A),
                        self.IK.get_LF_joint_angles(B, A),
                        self.IK.get_LR_joint_angles(B, A),
                        self.IK.get_RR_joint_angles(B, A)])
          
            # self.i += 1


    def LEGcallback(self, msg):
        # self.mis = msg.position
        self.get_logger().info(f'Received joint states:')
        # eulerAng = np.array([msg.euler_ang.x, msg.euler_ang.y, msg.euler_ang.z])
        # rf_coord = np.array([msg.rf.x, msg.rf.y, msg.rf.z])
        # lf_coord = np.array([msg.lf.x, msg.lf.y, msg.lf.z])
        # lr_coord = np.array([msg.lr.x, msg.lr.y, msg.lr.z])
        # rr_coord = np.array([msg.rr.x, msg.rr.y, msg.rr.z])

        eulerAng = np.array([0,0,0])
        rf_coord = np.array([0.13, 0.0, -0.24])
        lf_coord = np.array([0.13, 0.0, -0.24])
        lr_coord = np.array([0.13, 0.0, -0.24])
        rr_coord = np.array([0.13, 0.0, -0.24])

        self.ang_RF = self.IK.get_joint_angles(rf_coord, eulerAng)
        self.ang_LF = self.IK.get_joint_angles(lf_coord, eulerAng)
        self.ang_LR = self.IK.get_joint_angles(lr_coord, eulerAng)
        self.ang_RR = self.IK.get_joint_angles(rr_coord, eulerAng)

        print("debug")
        print(self.ang_LF)

  
        self.all_joint_angles = [self.ang_RF, self.ang_LF, self.ang_LR, self.ang_RR]
        print(self.all_joint_angles)

        self.target_received = True


    def SinPub(self, LegID = None):
        msg = JointTrajectory()

        joint_names = ["j_c1_rf", "j_thigh_rf", "j_tibia_rf",
                       "j_c1_lf", "j_thigh_lf", "j_tibia_lf",
                       "j_c1_lr", "j_thigh_lr", "j_tibia_lr",
                       "j_c1_rr", "j_thigh_rr", "j_tibia_rr"]


        # connected_leg = [joint_names[LegID*3 + 0], joint_names[LegID*3 + 1], joint_names[LegID*3 + 2]]
                       
                    
        sec = 0.5

        
        f = 0.08
        h = 0.24
        lift = 0.02
        span = 0.255
        self.repeat = 10
        ground = 0.05
        pathrange = 20

        L = []
        plot = []
        for j in range(pathrange):
            div = (f)/pathrange     
            x = span + div*j 
            z = -np.sqrt(lift**2 * (1 - (2*(x-span-f/2)/f)**2))
            plot.append([x, 0.0, z + ground])
            L.append(self.IK.get_joint_angles([x, 0.0, z + ground]))
        L.append(self.IK.get_joint_angles([span+f, 0.0, ground]))

        for j in range(pathrange):
            div = (f)/pathrange     
            x = span + f - div*j 
            L.append(self.IK.get_joint_angles([x, 0.0, ground]))
            plot.append([x, 0.0, ground])


        tar0 = self.IK.get_joint_angles([span + f/2, 0, -lift])
        tar1 = self.IK.get_joint_angles([span + f, 0, ground])
        tar2 = self.IK.get_joint_angles([span, 0, ground])

        tar3 = self.IK.get_joint_angles([span + f/2, 0, -lift], [0, 0, math.pi/4])
        tar4 = self.IK.get_joint_angles([span + f, 0, ground], [0, 0, math.pi/4])
        tar5 = self.IK.get_joint_angles([span, 0, ground], [0, 0, math.pi/4])




        L = [tar0, tar1, tar2] * self.repeat
        R = [tar4, tar4, tar4] * self.repeat
        # L = L*self.repeat

        seq = []
        for i in range(len(L)):
            seq.append(L[i]+R[i]+L[i]+L[i])
            # vel.append(vRF[i]+vRR[i]+vLR[i]+LF[i])

        points = []
        for i in range(len(seq)):
            point = JointTrajectoryPoint()
            point.time_from_start = Duration(seconds=(i+1)*sec, nanoseconds=0).to_msg()
            point.positions = seq[i]
            # point.velocities = vel[i]
            points.append(point)


        msg.joint_names = joint_names
        msg.points = points

        self.joint_publisher.publish(msg)


    def DuoPub(self, all_joint_angles = None):
        msg = JointTrajectory()

        joint_names = ["j_c1_rf", "j_thigh_rf", "j_tibia_rf",
                       "j_c1_lf", "j_thigh_lf", "j_tibia_lf",
                       "j_c1_lr", "j_thigh_lr", "j_tibia_lr",
                       "j_c1_rr", "j_thigh_rr", "j_tibia_rr"]
                       
                    
        sec = 0.1

        tar = self.IK.get_joint_angles([0.2, 0.0, 0.16])
        f = 0.05
        h = 0.24
        lift = 0.03
        span = 0.13
        self.repeat = 3


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

        seq = []
        for i in range(len(LF)):
            seq.append(RF[i]+LF[i]+LR[i]+RR[i])
            # vel.append(vRF[i]+vRR[i]+vLR[i]+LF[i])

        points = []
        for i in range(len(seq)):
            point = JointTrajectoryPoint()
            point.time_from_start = Duration(seconds=(i+1)*self.timer_period, nanoseconds=0).to_msg()
            point.positions = seq[i]
            # point.velocities = vel[i]
            points.append(point)


        msg.joint_names = joint_names
        msg.points = points

        self.joint_publisher.publish(msg)


    def TriPub(self, all_joint_angles = None):
        msg = JointTrajectory()

        joint_names = ["j_c1_rf", "j_thigh_rf", "j_tibia_rf",
                       "j_c1_lf", "j_thigh_lf", "j_tibia_lf",
                       "j_c1_lr", "j_thigh_lr", "j_tibia_lr",
                       "j_c1_rr", "j_thigh_rr", "j_tibia_rr"]
                       
                    
        sec = 0.1

        tar = self.IK.get_joint_angles([0.2, 0.0, 0.16])
        f = 0.05
        h = 0.24
        lift = 0.03
        span = 0.13
        self.repeat = 3


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

        seq = []
        for i in range(len(LF)):
            seq.append(RF[i]+LF[i]+LR[i]+RR[i])
            # vel.append(vRF[i]+vRR[i]+vLR[i]+LF[i])

        points = []
        for i in range(len(seq)):
            point = JointTrajectoryPoint()
            point.time_from_start = Duration(seconds=(i+1)*self.timer_period, nanoseconds=0).to_msg()
            point.positions = seq[i]
            # point.velocities = vel[i]
            points.append(point)


        msg.joint_names = joint_names
        msg.points = points

        self.joint_publisher.publish(msg)


    def pubpub(self, all_joint_angles = None):
        msg = JointTrajectory()

        joint_names = ["j_c1_rf", "j_thigh_rf", "j_tibia_rf",
                       "j_c1_lf", "j_thigh_lf", "j_tibia_lf",
                       "j_c1_lr", "j_thigh_lr", "j_tibia_lr",
                       "j_c1_rr", "j_thigh_rr", "j_tibia_rr"]
                       
                    
        sec = 0.1

        tar = self.IK.get_joint_angles([0.2, 0.0, 0.16])
        f = 0.05
        h = 0.24
        lift = 0.03
        span = 0.13
        self.repeat = 3

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

        seq = []
        for i in range(len(LF)):
            seq.append(RF[i]+LF[i]+LR[i]+RR[i])
            # vel.append(vRF[i]+vRR[i]+vLR[i]+LF[i])

        points = []
        for i in range(len(seq)):
            point = JointTrajectoryPoint()
            point.time_from_start = Duration(seconds=(i+1)*self.timer_period, nanoseconds=0).to_msg()
            point.positions = seq[i]
            # point.velocities = vel[i]
            points.append(point)


        msg.joint_names = joint_names
        msg.points = points

        self.joint_publisher.publish(msg)

        


def main(args=None):
    rclpy.init(args=args)

    joint_publisher = JointPublisher()

    # joint_publisher.pubpub()
    joint_publisher.SinPub()

    # rclpy.spin_once(joint_publisher)

    joint_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
