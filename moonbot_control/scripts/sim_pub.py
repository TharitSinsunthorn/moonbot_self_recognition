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


        ##### TIMER ######
        self.timer_period = 3
        self.timer = self.create_timer(self.timer_period, self.pub_callback)
        ##### TIMER ######

        self.IK = InvKinematics()

        ##### Gait Parameter #####
        self.swing_sample = 20
        self.stance_sample = 20
        self.step_len = 0.05
        self.step_height = 0.06
        ##### Gait Parameter #####

        ##### Pose Parameters #####
        self.span = 0.13 * np.sin(math.pi/4)
        self.height = 0.24

        # self.EEpose = np.zeros([4,3]) #End Effector position
        # for i in range(4):
        #     self.EEpose[i,:] = np.array([self.span * np.cos((2*i+1)*math.pi/4), self.span * np.sin((2*i+1)*math.pi/4), self.height]) 

        # self.TRAJ = np.zeros([4,3])
        # self.ZMP = np.zeros([4,3])

        self.RF_pose = [[self.span, self.span, self.height]]
        self.LF_pose = [[-self.span, self.span, self.height]]
        self.LR_pose = [[-self.span, -self.span, self.height]]
        self.RR_pose = [[self.span, -self.span, self.height]]
        ##### Pose Parameters #####

        self.ang_RF = []
        self.ang_LF = []
        self.ang_LR = []
        self.ang_RR = []
        self.all_joint_angles = []


    def zmp_handler(self, shift):
        RF_zmp = np.array(self.RF_pose[0]) + np.array(shift)
        LF_zmp = np.array(self.LF_pose[0]) + np.array(shift)
        LR_zmp = np.array(self.LR_pose[0]) + np.array(shift)
        RR_zmp = np.array(self.RR_pose[0]) + np.array(shift)

        self.RF_pose.append(RF_zmp.tolist())
        self.LF_pose.append(LF_zmp.tolist())
        self.LR_pose.append(LR_zmp.tolist())
        self.RR_pose.append(RR_zmp.tolist())


    def swing_RF(self):
        trajRF = []
        for j in range(0, self.swing_sample):
            div = self.step_len / self.swing_sample     
            x = 0.0
            y = div*j
            z = - self.step_height*np.sin(math.pi/self.step_len * y)
            traj.append(self.IK.get_RF_joint_angles([x, y, z]))

    def stance_RF(self):
        trajRF = []
        for j in range(0,self.stance_sample, 2):
            div = (f)/pathrange     
            x = span + f - div*j 
            traj.append(self.IK.get_joint_angles([x, 0.0, ground]))
            plot.append([x, 0.0, ground])


    def crawl_gait(self):
        self.zmp_handler([-0.03, 0.0, 0.0])
    
    def pub_callback(self):

        self.crawl_gait()

        for i in range(len(self.RF_pose)):
            self.ang_RF.append(self.IK.get_RF_joint_angles(self.RF_pose[i], [0,0,0]))
            self.ang_LF.append(self.IK.get_LF_joint_angles(self.LF_pose[i], [0,0,0]))
            self.ang_LR.append(self.IK.get_LR_joint_angles(self.LR_pose[i], [0,0,0]))
            self.ang_RR.append(self.IK.get_RR_joint_angles(self.RR_pose[i], [0,0,0]))

        self.RobotPub()

        self.get_logger().info(f'pub')

        self.ang_RF.clear()
        self.ang_LF.clear()
        self.ang_LR.clear()
        self.ang_RR.clear()


 
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

        sec = 1

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