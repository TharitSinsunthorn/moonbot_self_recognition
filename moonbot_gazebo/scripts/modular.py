#! /usr/bin/env python3

import sys
import os
import csv
import rclpy
from rclpy.duration import Duration 
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

from IK.limb_kinematics import InvKinematics
import IK.params as params


class LimbActionClient(Node):

    def __init__(self):
        super().__init__("limb_actionclient")

        self.group = ReentrantCallbackGroup()

        self._action_RF = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/RF/position_trajectory_controller/follow_joint_trajectory',
            callback_group=self.group)

        self._action_RR = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/RR/position_trajectory_controller/follow_joint_trajectory',
            callback_group=self.group)

        self._action_LR = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/LR/position_trajectory_controller/follow_joint_trajectory',
            callback_group=self.group)

        self._action_LF = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/LF/position_trajectory_controller/follow_joint_trajectory',
            callback_group=self.group)
        
        self.IK = InvKinematics()
        self.repeat = 3

        self.csv_file_path = '../moonbot_ws/src/moonbot_gazebo/src/2data.csv'
        self.seq = []

        self.span = params.span
        self.height = params.height


    def seq_generator(self, leg_no):
        gen_seq = []
        with open(self.csv_file_path, 'r') as file:
            csv_reader = csv.reader(file)
            data_list = []

            for row in csv_reader:
                data_list.append(row)

        # for i in range(12):
        for j in range(1,8002):
            if leg_no == "LF":
                float_generator = [float(item) for item in data_list[j][111:114]]
            elif leg_no == "LR":
                float_generator = [float(item) for item in data_list[j][114:117]]
            elif leg_no == "RR":
                float_generator = [float(item) for item in data_list[j][117:120]]
            elif leg_no == "RF":
                float_generator = [float(item) for item in data_list[j][120:123]]

            gen_seq.append(float_generator)

        return gen_seq


    def send_goal(self):
        RF_goal_msg = FollowJointTrajectory.Goal()
        RR_goal_msg = FollowJointTrajectory.Goal()
        LR_goal_msg = FollowJointTrajectory.Goal()
        LF_goal_msg = FollowJointTrajectory.Goal()



        # Fill in data for trajectory
        joint_names_rf = ["j_c1_rf", "j_thigh_rf", "j_tibia_rf"]
        joint_names_rr = ["j_c1_rr", "j_thigh_rr", "j_tibia_rr"]
        joint_names_lr = ["j_c1_lr", "j_thigh_lr", "j_tibia_lr"]
        joint_names_lf = ["j_c1_lf", "j_thigh_lf", "j_tibia_lf"]
                       

        
        sec = 0.7  

        f = -0.04
        h = self.height
        lift = 0.03
        span = self.span

        startconfig = self.IK.get_joint_angles([span, 0.0, 0.1])
        startconfig2 = self.IK.get_joint_angles([span, 0.0, h])

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

        # mission = "w"
        # if mission == "s":
        #     print("stand up!")
        #     # standup seq
        #     LF = [startconfig, startconfig, startconfig2]
        #     RF = [startconfig, startconfig, startconfig2]
        #     RR = [startconfig, startconfig, startconfig2]
        #     LR = [startconfig, startconfig, startconfig2]

        # elif mission == "w":
        #     print("walk!")
        #     # Move forward
        #     RF = [tar1, tar2, tar3]*repeat
        #     LR = [tar4, tar5, tar6]*repeat
        #     LF = [tar7, tar8, tar9]*repeat
        #     RR = [tar10, tar11, tar12]*repeat

        #     # Initial and end forward
        #     RF.insert(0, RF[0])
        #     RF.append(startconfig2)

        #     LR.insert(0, LR[0])
        #     LR.append(startconfig2)

        #     LF.insert(0, LF[-1])
        #     LF.append(startconfig2)

        #     RR.insert(0, RR[-1])
        #     RR.append(startconfig2)

        # elif mission == "l":
        #     print("go left!")
        #     # Move Left
        #     RR = [tar1, tar2, tar3]*repeat
        #     LF = [tar4, tar5, tar6]*repeat
        #     RF = [tar7, tar8, tar9]*repeat
        #     LR = [tar10, tar11, tar12]*repeat

        #     # Initial left
        #     RR.insert(0, RR[0])
        #     RR.append(tar4)

        #     LF.insert(0, LF[0])
        #     LF.append(tar4)

        #     RF.insert(0, RF[-1])
        #     RF.append(tar4)

        #     LR.insert(0, LR[-1])
        #     LR.append(tar4)

        

        RF = [tar1, tar2, tar3, tar4, tar5, tar5]*self.repeat
        LR = [tar6, tar7, tar8, tar9, tar10,tar10]*self.repeat
        LF = [tar11, tar11, tar12, tar13, tar14, tar15]*self.repeat
        RR = [tar16, tar16, tar17, tar18, tar19, tar20]*self.repeat

        RF.insert(0, RF[-1])
        RF.insert(0, RF[-1])

        LR.insert(0, LR[-1])
        LR.insert(0, LR[-1])

        LF.insert(0, LF[-2])
        LF.insert(0, LF[-1])
        LF[-2] = tar19
        LF[-1] = tar4

        RR.insert(0, RR[-2])
        RR.insert(0, RR[-1])
        RR[-2] = tar14
        RR[-1] = tar4


        # Go left
        # RR = [tar1, tar2, tar3, tar4, tar5, tar5]*self.repeat
        # LF = [tar6, tar7, tar8, tar9, tar10,tar10]*self.repeat
        # RF = [tar11, tar11, tar12, tar13, tar14, tar15]*self.repeat
        # LR = [tar16, tar16, tar17, tar18, tar19, tar20]*self.repeat

        # RR.insert(0, RR[-1])
        # RR.insert(0, RR[-1])

        # LF.insert(0, LF[-1])
        # LF.insert(0, LF[-1])

        # RF.insert(0, RF[-2])
        # RF.insert(0, RF[-1])
        # RF[-2] = tar19
        # RF[-1] = tar4

        # LR.insert(0, LR[-2])
        # LR.insert(0, LR[-1])
        # LR[-2] = tar14
        # LR[-1] = tar4


        RF_points = []
        RR_points = []
        LR_points = []
        LF_points = []
        for i in range(len(RF)):
            RF_point = JointTrajectoryPoint()
            RR_point = JointTrajectoryPoint()
            LR_point = JointTrajectoryPoint()
            LF_point = JointTrajectoryPoint()

            RF_point.time_from_start = Duration(seconds=(i+1)*sec, nanoseconds=0).to_msg()
            RR_point.time_from_start = Duration(seconds=(i+1)*sec, nanoseconds=0).to_msg()
            LR_point.time_from_start = Duration(seconds=(i+1)*sec, nanoseconds=0).to_msg()
            LF_point.time_from_start = Duration(seconds=(i+1)*sec, nanoseconds=0).to_msg()

            RF_point.positions = RF[i]
            RR_point.positions = RR[i]
            LR_point.positions = LR[i]
            LF_point.positions = LF[i]

            # point.velocities = vel[i]
            RF_points.append(RF_point)
            RR_points.append(RR_point)
            LR_points.append(LR_point)
            LF_points.append(LF_point)
            # print(points)

        RF_goal_msg.goal_time_tolerance = Duration(seconds=1, nanoseconds=0).to_msg()
        RF_goal_msg.trajectory.joint_names = joint_names_rf
        RF_goal_msg.trajectory.points = RF_points

        RR_goal_msg.goal_time_tolerance = Duration(seconds=1, nanoseconds=0).to_msg()
        RR_goal_msg.trajectory.joint_names = joint_names_rr
        RR_goal_msg.trajectory.points = RR_points

        LR_goal_msg.goal_time_tolerance = Duration(seconds=1, nanoseconds=0).to_msg()
        LR_goal_msg.trajectory.joint_names = joint_names_lr
        LR_goal_msg.trajectory.points = LR_points

        LF_goal_msg.goal_time_tolerance = Duration(seconds=1, nanoseconds=0).to_msg()
        LF_goal_msg.trajectory.joint_names = joint_names_lf
        LF_goal_msg.trajectory.points = LF_points

        self._action_RF.wait_for_server()
        self._sendRF_goal_future = self._action_RF.send_goal_async(
            RF_goal_msg, feedback_callback=self.feedback_callback)
        self._sendRF_goal_future.add_done_callback(self.goal_response_callback)

        self._action_RR.wait_for_server()
        self._sendRR_goal_future = self._action_RR.send_goal_async(
            RR_goal_msg, feedback_callback=self.feedback_callback)
        self._sendRR_goal_future.add_done_callback(self.goal_response_callback)

        self._action_LR.wait_for_server()
        self._sendLR_goal_future = self._action_LR.send_goal_async(
            LR_goal_msg, feedback_callback=self.feedback_callback)
        self._sendLR_goal_future.add_done_callback(self.goal_response_callback)

        self._action_LF.wait_for_server()
        self._sendLF_goal_future = self._action_LF.send_goal_async(
            LF_goal_msg, feedback_callback=self.feedback_callback)
        self._sendRR_goal_future.add_done_callback(self.goal_response_callback)


    def send_goal2(self):
        RF_goal_msg = FollowJointTrajectory.Goal()
        RR_goal_msg = FollowJointTrajectory.Goal()
        LR_goal_msg = FollowJointTrajectory.Goal()
        LF_goal_msg = FollowJointTrajectory.Goal()


        # Fill in data for trajectory
        joint_names_rf = ["j_c1_rf", "j_thigh_rf", "j_tibia_rf"]
        joint_names_rr = ["j_c1_rr", "j_thigh_rr", "j_tibia_rr"]
        joint_names_lr = ["j_c1_lr", "j_thigh_lr", "j_tibia_lr"]
        joint_names_lf = ["j_c1_lf", "j_thigh_lf", "j_tibia_lf"]
                       

        sec = 0.0015

        f = -0.04
        h = 0.26
        lift = 0.03
        span = 0.11

        RR = self.seq_generator("RR")*self.repeat
        LF = self.seq_generator("LF")*self.repeat
        RF = self.seq_generator("RF")*self.repeat
        LR = self.seq_generator("LR")*self.repeat

        print(len(LF))



        RF_points = []
        RR_points = []
        LR_points = []
        LF_points = []
        for i in range(len(RF)):
            RF_point = JointTrajectoryPoint()
            RR_point = JointTrajectoryPoint()
            LR_point = JointTrajectoryPoint()
            LF_point = JointTrajectoryPoint()

            if i == 0:
                RF_point.time_from_start = Duration(seconds=1, nanoseconds=0).to_msg()
                RR_point.time_from_start = Duration(seconds=1, nanoseconds=0).to_msg()
                LR_point.time_from_start = Duration(seconds=1, nanoseconds=0).to_msg()
                LF_point.time_from_start = Duration(seconds=1, nanoseconds=0).to_msg()
            else:

                RF_point.time_from_start = Duration(seconds=(i+1)*sec + 1, nanoseconds=0).to_msg()
                RR_point.time_from_start = Duration(seconds=(i+1)*sec + 1, nanoseconds=0).to_msg()
                LR_point.time_from_start = Duration(seconds=(i+1)*sec + 1, nanoseconds=0).to_msg()
                LF_point.time_from_start = Duration(seconds=(i+1)*sec + 1, nanoseconds=0).to_msg()

            RF_point.positions = RF[i]
            RR_point.positions = RR[i]
            LR_point.positions = LR[i]
            LF_point.positions = LF[i]

            # point.velocities = vel[i]
            RF_points.append(RF_point)
            RR_points.append(RR_point)
            LR_points.append(LR_point)
            LF_points.append(LF_point)
            # print(points)

        RF_goal_msg.goal_time_tolerance = Duration(seconds=1, nanoseconds=0).to_msg()
        RF_goal_msg.trajectory.joint_names = joint_names_rf
        RF_goal_msg.trajectory.points = RF_points

        RR_goal_msg.goal_time_tolerance = Duration(seconds=1, nanoseconds=0).to_msg()
        RR_goal_msg.trajectory.joint_names = joint_names_rr
        RR_goal_msg.trajectory.points = RR_points

        LR_goal_msg.goal_time_tolerance = Duration(seconds=1, nanoseconds=0).to_msg()
        LR_goal_msg.trajectory.joint_names = joint_names_lr
        LR_goal_msg.trajectory.points = LR_points

        LF_goal_msg.goal_time_tolerance = Duration(seconds=1, nanoseconds=0).to_msg()
        LF_goal_msg.trajectory.joint_names = joint_names_lf
        LF_goal_msg.trajectory.points = LF_points

        self._action_RF.wait_for_server()
        self._sendRF_goal_future = self._action_RF.send_goal_async(
            RF_goal_msg, feedback_callback=self.feedback_callback)
        self._sendRF_goal_future.add_done_callback(self.goal_response_callback)

        self._action_RR.wait_for_server()
        self._sendRR_goal_future = self._action_RR.send_goal_async(
            RR_goal_msg, feedback_callback=self.feedback_callback)
        self._sendRR_goal_future.add_done_callback(self.goal_response_callback)

        self._action_LR.wait_for_server()
        self._sendLR_goal_future = self._action_LR.send_goal_async(
            LR_goal_msg, feedback_callback=self.feedback_callback)
        self._sendLR_goal_future.add_done_callback(self.goal_response_callback)

        self._action_LF.wait_for_server()
        self._sendLF_goal_future = self._action_LF.send_goal_async(
            LF_goal_msg, feedback_callback=self.feedback_callback)
        self._sendRR_goal_future.add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        
        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)


    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: '+str(result))
        rclpy.shutdown()


    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # self.get_logger().info('Received feedbackl:'+str(feedback))

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        
        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)


    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: '+str(result))
        rclpy.shutdown()


    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # self.get_logger().info('Received feedbackl:'+str(feedback))


def main(args=None):
    rclpy.init()

    # action_client = LimbActionClient()
    try:
        action_client = LimbActionClient()
        action_client.send_goal2()


        executor = MultiThreadedExecutor()
        executor.add_node(action_client)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            action_client.destroy_node()

    finally:
        rclpy.shutdown()


    # action_client.send_goal()
    
    # rclpy.spin(action_client)

if __name__ == '__main__':
    main()

