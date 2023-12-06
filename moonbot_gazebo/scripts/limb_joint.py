#! /usr/bin/env python3

import sys
import os
import rclpy
from rclpy.duration import Duration 
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from IK.limb_kinematics import InvKinematics
import IK.params as params
# ros2 action list -t
# ros2 action info /position_trajectory_controller/follow_joint_trajectory -t
# ros2 interface show control_msgs/action/FolowJointTrajectory

# sys.path.append('../moonbot_ws/src/moonbot_gazebo/src')
import csv

class LimbActionClient(Node):

    def __init__(self):
        super().__init__("limb_actionclient")
        self._action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/position_trajectory_controller/follow_joint_trajectory')
        
        self.IK = InvKinematics()
        self.repeat = 3

        self.csv_file_path = '../moonbot_ws/src/moonbot_gazebo/src/002legh_Crawl.csv'
        self.seq = []

        self.span = params.span
        self.height = params.height

        

    def seq_generator(self):

        with open(self.csv_file_path, 'r') as file:
            csv_reader = csv.reader(file)
            data_list = []

            for row in csv_reader:
                data_list.append(row)
            # print(data_list[1][111:])
            # print("\n")
            # print(data_list[-1][111:])

        # for i in range(12):
        for j in range(1,len(data_list)):
            # print(data_list[j][111:])
            float_generator = [float(item) for item in data_list[j][111:]]
            self.seq.append(float_generator)

        return self.seq


    def send_goal(self):
        goal_msg = FollowJointTrajectory.Goal()

        # Fill in data for trajectory
        joint_names = ["j_c1_lf", "j_thigh_lf", "j_tibia_lf",
                       "j_c1_lr", "j_thigh_lr", "j_tibia_lr",
                       "j_c1_rr", "j_thigh_rr", "j_tibia_rr",
                       "j_c1_rf", "j_thigh_rf", "j_tibia_rf",]


                       
        sec = 0.001

        tar = self.IK.get_joint_angles([0.2, 0.0, 0.16])
        f = 0.05
        h = 0.24
        lift = 0.03
        span = self.span

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

        # standup seq
        # LF = [[0.0, 0.756, -1.57], tar]
        # RF = [[0.0, 0.756, -1.57], tar]
        # RR = [[0.0, 0.756, -1.57], tar]
        # LR = [[0.0, 0.756, -1.57], tar]


        RF = [tar1, tar2, tar3, tar4, tar5, tar5]*self.repeat
        LR = [tar6, tar7, tar8, tar9, tar10,tar10]*self.repeat
        RR = [tar11, tar11, tar12, tar13, tar14, tar15]*self.repeat
        LF = [tar16, tar16, tar17, tar18, tar19, tar20]*self.repeat
              

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

       
        
        seq = []
        vel = []
        for i in range(len(LF)):
            seq.append(LF[i]+RF[i]+RR[i]+LR[i])
            # vel.append(vLF[i]+vRF[i]+vRR[i]+vLR[i])

        seq_ori = self.seq_generator()
        # vel = vel*self.repeat
        seq = seq_ori*self.repeat
        # seq.insert(0, tar9+tar1+tar12+tar4)
        # seq.append(tar1+tar1+tar1+tar1)

        # print(seq)

        # point1.positions = [0.0, 0.0, 0.0]
        points = []
        

        for i in range(len(seq)):
            point = JointTrajectoryPoint()
            
            point.time_from_start = Duration(seconds=(i)*sec + 1, nanoseconds=0).to_msg()
            # point.time_from_start = Duration(seconds=(i)*sec + 1 + 0.45*(i//len(seq_ori)), nanoseconds=0).to_msg()
            point.positions = seq[i]
            # point.velocities = vel[i]
            points.append(point)
            # print(points)

        goal_msg.goal_time_tolerance = Duration(seconds=1, nanoseconds=0).to_msg()
        goal_msg.trajectory.joint_names = joint_names
        goal_msg.trajectory.points = points

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)

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
        self.get_logger().info('Received feedbackl:'+str(feedback.error.positions)+str(type(feedback.actual.positions)))


def main(args=None):
    rclpy.init()

    action_client = LimbActionClient()

    # angle = [1.0, 1.0]
    # print(self.IK([0.1, 0.0, 0.1]))
    action_client.send_goal()
    
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()

