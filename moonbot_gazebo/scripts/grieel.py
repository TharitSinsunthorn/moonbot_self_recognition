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
# ros2 action list -t
# ros2 action info /position_trajectory_controller/follow_joint_trajectory -t
# ros2 interface show control_msgs/action/FolowJointTrajectory


class LimbActionClient(Node):

    def __init__(self):
        super().__init__("limb_actionclient")
        self._action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/RR/position_trajectory_controller/follow_joint_trajectory')
        
        self.IK = InvKinematics()
        self.repeat = 1


    def send_goal(self):
        goal_msg = FollowJointTrajectory.Goal()

        # Fill in data for trajectory

        joint_names = ["j_c1_gr", "j_thigh_gr", "j_end_gr"]
        
        sec = 0.25

        # gripper seq
        GR = [[0.0, 0.7, 0.756],
              [0.0, 0.7, 0.756],
              [0.0, 0.7, 0.756],
              [0.3, 0.7, 0.756],
              [-0.3, 0.7, 0.756],
              [0.3, 0.7, 0.756],
              [-0.3, 0.7, 0.756],
              [0.0, 0.7, 0.756],
              [0.0, 0.7, 0.756],
              [0.0, 0.7, 1.56],
              [0.0, 0.7, 0.756],
              [0.0, 0.7, 1.56],
              [0.0, 0.7, 0.756]]

        points = []
        

        for i in range(len(GR)):
            point = JointTrajectoryPoint()
            point.time_from_start = Duration(seconds=(i+1)*sec, nanoseconds=0).to_msg()
            point.positions = GR[i]
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