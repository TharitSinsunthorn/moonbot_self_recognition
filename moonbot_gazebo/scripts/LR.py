#!/usr/bin/env python3

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


class LimbActionClient(Node):

    def __init__(self):
        super().__init__("limb_actionclient")
        self._action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/LR/position_trajectory_controller/follow_joint_trajectory')
        
        self.IK = InvKinematics()
        self.repeat = 1

        self.span = params.span
        self.height = params.height


    def send_goal(self):
        goal_msg = FollowJointTrajectory.Goal()
        
        joint_names = ["j_c1_lr", "j_thigh_lr", "j_tibia_lr"]
        
        sec = 1.0

        f = -0.04
        h = 0.24
        tar = self.IK.get_joint_angles([self.span, 0.0, self.height])
        targ = self.IK.get_joint_angles([0.09, 0.05, h])

        # standup seq
        LR = [targ]

        # Gait
        tar4 = self.IK.get_joint_angles([0.13, 0.0, h])
        tar5 = self.IK.get_joint_angles([0.13-f/2, -f/2, h-0.05])
        tar6 = self.IK.get_joint_angles([0.13-f, -f, h])

        # LR = [tar4, tar5, tar6]

    
        # seq = [LF[0]+RF[0]+RR[0]+LR[0], LF[1]+RF[1]+RR[1]+LR[1]]
        seq = []
        for i in range(len(LR)):
            seq.append(LR[i])
        seq = seq*self.repeat


        vRF = [[0.0, 1.0, 1.0], [0.0, 0.0, 0.0]]
        v = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]

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

    # angle = [1.0, 1.0]
    action_client.send_goal()
    
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()