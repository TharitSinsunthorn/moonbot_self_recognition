#! /usr/bin/env python3

import sys
import os
import rclpy
from rclpy.duration import Duration 
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

import numpy as np
from IK.limb_kinematics import InvKinematics
# ros2 action list -t
# ros2 action info /position_trajectory_controller/follow_joint_trajectory -t
# ros2 interface show control_msgs/action/FolowJointTrajectory


class LimbActionClient(Node):

    def __init__(self):
        super().__init__("limb_actionclient")
        self._action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/position_trajectory_controller/follow_joint_trajectory')
        
        self.RFsub = self.create_subscription(JointState, 
            "RFstate", 
            self.callback, 10)
        self.testsub

        self.IK = InvKinematics()

        self.tar = []

    def callback(self, msg):
        # self.mis = msg.position
        self.get_logger().info(f'Received joint states: {msg.position}')
        self.tar = msg.position



    def send_goal(self):
        goal_msg = FollowJointTrajectory.Goal()

        # Fill in data for trajectory
        joint_names = ["j_c1_rf", "j_thigh_rf", "j_tibia_rf",
                       "j_c1_rr", "j_thigh_rr", "j_tibia_rr",
                       "j_c1_lr", "j_thigh_lr", "j_tibia_lr",
                       "j_c1_lf", "j_thigh_lf", "j_tibia_lf",]
        
        sec = 2.0

        # tar = lk.inverse_kinematics([0.19, 0.0, 0.16])

        
        f = -0.06
        h = 0.24
        tar0 = self.IK.get_joint_angles([0.13, 0.0, -0.1])
        tar1 = self.IK.get_joint_angles([0.13, 0.0, h])
        sit = self.IK.get_joint_angles([0.13, 0.0, 0.1])

        test = self.IK.get_joint_angles(np.array([0.13, 0.0, h]))
        # print(test)
        # print(tar1)

        # standup seq
        RF = [tar0,tar0,tar1]
        RR = [tar0,tar0,tar1]
        LR = [tar0,tar0,tar1]
        LF = [tar0,tar0,tar1]


        # emergency
        E = [[0.0, -1.0, -0.57], 
            [0.3, -0.6, -0.8], 
            [0.6, -1.0, -0.57],
            [0.0, -1.0, -0.57]]
              

        vLF = [[0.0, 0.0, 0.0]]

        vRF = [[0.0, 0.0, 0.0]]
              

        vRR = [[0.0, 0.0, 0.0]]

        vLR = [[0.0, 0.0, 0.0]]
        

        seq = []
        vel = []
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
        # self.get_logger().info('Received feedbackl:'+str(feedback))


def main(args=None):
    rclpy.init()

    action_client = LimbActionClient()

    action_client.send_goal()
    rclpy.spin(action_client)

    
    

if __name__ == '__main__':
    main()

