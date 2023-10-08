#! /usr/bin/env python3

import sys
import rclpy
from rclpy.duration import Duration 
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

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
        
    def send_goal(self):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg2 = FollowJointTrajectory.Goal()

        # Fill in data for trajectory
        joint_names = ["j_c1_rf", "j_c1_lr", "j_c1_rr",
                       "j_thigh_rf", "j_thigh_lr", "j_thigh_rr",
                       "j_tibia_rf", "j_tibia_lr", "j_tibia_rr"]

        joint_names2 = ["j_c1_lf", "j_thigh_lf", "j_tibia_lf"]
        
        sec = 1.0


        points = []
        point1 = JointTrajectoryPoint()
        point1.positions = [0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 
                            0.0, 0.0, 0.0]

        point2 = JointTrajectoryPoint()
        point2.time_from_start = Duration(seconds=sec, nanoseconds=0).to_msg()
        point2.positions = [0.0, 0.0, 0.25,
                            0.0, 0.0, -0.5, 
                            0.0, 0.0, 0.0]

        point3 = JointTrajectoryPoint()
        point3.time_from_start = Duration(seconds=2*sec, nanoseconds=0).to_msg()
        point3.positions = [0.0, 0.0, 0.5,
                            0.0, 0.0, 0.0,  
                            0.0, 0.0, 0.0]
        
        sings = []
        sing1 = JointTrajectoryPoint()
        sing1.positions = [0.0, 0.0, 0.0]
                            
        sing2 = JointTrajectoryPoint()
        sing2.time_from_start = Duration(seconds=sec, nanoseconds=0).to_msg()
        sing2.positions = [0.0, 0.0, 0.25]

        sing3 = JointTrajectoryPoint()
        sing3.time_from_start = Duration(seconds=2*sec, nanoseconds=0).to_msg()
        sing3.positions = [0.0, 0.0, 0.5]

        for i in range(3):
            point_name = f"point{i+1}"  # Generate the variable name using f-string
            point = eval(point_name)
            points.append(point)

            sing_name = f"sing{i+1}"
            sing = eval(sing_name)
            sings.append(sing)


        goal_msg.goal_time_tolerance = Duration(seconds=1, nanoseconds=0).to_msg()
        goal_msg.trajectory.joint_names = joint_names
        goal_msg.trajectory.points = points

        goal_msg2.goal_time_tolerance = Duration(seconds=1, nanoseconds=0).to_msg()
        goal_msg2.trajectory.joint_names = joint_names2
        goal_msg2.trajectory.points = sings

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

        # self._send_goal_future = self._action_client.send_goal_async(
        #     goal_msg2, feedback_callback=self.feedback_callback)

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

