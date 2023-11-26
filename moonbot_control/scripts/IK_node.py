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

        self.IK = InvKinematics()

        self.tar = []

        self.target_received = False

        self.ang_RF = []
        self.ang_LF = []
        self.ang_LR = []
        self.ang_RR = []


    def LEGcallback(self, msg):
        # self.mis = msg.position
        self.get_logger().info(f'Received joint states:')
        # eulerAng = np.array([msg.euler_ang.x, msg.euler_ang.y, msg.euler_ang.z])
        # rf_coord = np.array([msg.rf.x, msg.rf.y, msg.rf.z])
        # lf_coord = np.array([msg.lf.x, msg.lf.y, msg.lf.z])
        # lr_coord = np.array([msg.lr.x, msg.lr.y, msg.lr.z])
        # rr_coord = np.array([msg.rr.x, msg.rr.y, msg.rr.z])

        eulerAng = np.array([0,0,0])
        rf_coord = np.array([0.13, 0.0, 0.24])
        lf_coord = np.array([0.13, 0.0, 0.24])
        lr_coord = np.array([0.13, 0.0, 0.24])
        rr_coord = np.array([0.13, 0.0, 0.24])

        self.ang_RF = self.IK.get_joint_angles(rf_coord, eulerAng)
        self.ang_LF = self.IK.get_joint_angles(lf_coord, eulerAng)
        self.ang_LR = self.IK.get_joint_angles(lr_coord, eulerAng)
        self.ang_RR = self.IK.get_joint_angles(rr_coord, eulerAng)

        print("debug")
        print(self.ang_LF)

        # self.ang_RF = self.ang_RF.tolist()
        # self.ang_LF = self.ang_LF.tolist()
        # self.ang_LR = self.ang_LR.tolist()
        # self.ang_RR = self.ang_RR.tolist()


        # self.tar = msg.position
        # self.tar = self.tar.tolist()
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
                       "j_c1_lf", "j_thigh_lf", "j_tibia_lf",
                       "j_c1_lr", "j_thigh_lr", "j_tibia_lr",
                       "j_c1_rr", "j_thigh_rr", "j_tibia_rr"]
                       
                       

        sec = 2.0

        f = -0.06
        h = 0.24
        tar0 = self.IK.get_joint_angles([0.13, 0.0, -0.1])
        tar1 = self.IK.get_joint_angles([0.13, 0.0, h])
        sit = self.IK.get_joint_angles([0.13, 0.0, 0.1])

        test = self.IK.get_joint_angles(np.array([0.13, 0.0, h]))
        # print(self.tar)

        # standup seq
        # RF = [tar0,tar0,self.tar]
        # RR = [tar0,tar0,tar1]
        # LR = [tar0,tar0,tar1]
        # LF = [tar0,tar0,tar1]

        RF = [self.ang_RF]
        LF = [self.ang_LF]
        LR = [self.ang_LR]
        RR = [self.ang_RR]

        seq = []
        for i in range(len(LF)):
            seq.append(RF[i]+LF[i]+LR[i]+RR[i])
            # vel.append(vRF[i]+vRR[i]+vLR[i]+LF[i])
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

    joint_publisher.target_checker()

    rclpy.spin(joint_publisher)

    joint_publisher.destroy_node()

    rclpy.shutdown()

# def main(args=None):
#     rclpy.init(args=args)
#     iknode = IKNode()
#     executor = SingleThreadedExecutor()
#     executor.add_node(iknode)
#     try:
#         executor.spin()
#     except KeyboardInterrupt as e:
#         iknode.get_logger().debug('KeyboardInterrupt caught, node shutting down cleanly\nbye bye <3')
#     iknode.destroy_node()
#     rclpy.shutdown()


if __name__ == '__main__':
    main()


