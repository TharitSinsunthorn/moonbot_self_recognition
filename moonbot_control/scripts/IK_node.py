#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moonbot_custom_interfaces.msg import Geometry

import numpy as np
from IK.limb_kinematics import InvKinematics

class JointPublisher(Node):
    def __init__(self):
        super().__init__('JointPublisher_Node')

        self.group = rclpy.callback_groups.ReentrantCallbackGroup()
        
        
        ##### PUBLISHER #####
        command_topics = [
            'RFposition_trajectory_controller/joint_trajectory',
            'LFposition_trajectory_controller/joint_trajectory',
            'LRposition_trajectory_controller/joint_trajectory',
            'RRposition_trajectory_controller/joint_trajectory',
        ]

        self.joints_pub = [self.create_publisher(JointTrajectory, topic, 1, callback_group=self.group) for topic in command_topics]
        ##### PUBLISHER #####
        
        ##### SUBSCRIBER #####
        self.create_subscription(Geometry, "moonbot_geometry", self.LEGcallback, 1)
        ##### SUBSCRIBER #####
        
        ##### TIMER #####
        self.timer_period = 0.1
        self.create_timer(self.timer_period, self.pub_callback)
        ##### TIMER #####

        self.IK = InvKinematics()

        self.target_received = False
        self.joint_names = {
            'RF': ["j_c1_rf", "j_thigh_rf", "j_tibia_rf"],
            'LF': ["j_c1_lf", "j_thigh_lf", "j_tibia_lf"],
            'LR': ["j_c1_lr", "j_thigh_lr", "j_tibia_lr"],
            'RR': ["j_c1_rr", "j_thigh_rr", "j_tibia_rr"],
        }

    def LEGcallback(self, msg):
        """Callback for end-effector position subscriber

        Args:
            msg: moonbot_custom_interfaces.msg.Geometry
        """
        
        eulerAng = np.array([msg.euler_ang.x, msg.euler_ang.y, msg.euler_ang.z])
        coords = {
            'RF': np.array([msg.rf.x, msg.rf.y, msg.rf.z]),
            'LF': np.array([msg.lf.x, msg.lf.y, msg.lf.z]),
            'LR': np.array([msg.lr.x, msg.lr.y, msg.lr.z]),
            'RR': np.array([msg.rr.x, msg.rr.y, msg.rr.z]),
        }
        
        self.angles = {
            'RF': self.IK.get_RF_joint_angles(coords['RF'], eulerAng),
            'LF': self.IK.get_LF_joint_angles(coords['LF'], eulerAng),
            'LR': self.IK.get_LR_joint_angles(coords['LR'], eulerAng),
            'RR': self.IK.get_RR_joint_angles(coords['RR'], eulerAng),
        }

        self.target_received = True

    def pub_callback(self):
        if self.target_received:
            self.publish_joint_angles()

    def publish_joint_angles(self):
        sec = self.timer_period
        for i, (limb, angles) in enumerate(self.angles.items()):
            msg = JointTrajectory()
            point = JointTrajectoryPoint()
            point.time_from_start = Duration(seconds=sec).to_msg()
            point.positions = angles

            msg.joint_names = self.joint_names[limb]
            msg.points = [point]

            self.joints_pub[i].publish(msg)

def main(args=None):
    rclpy.init(args=args)
    joint_publisher = JointPublisher()
    rclpy.spin(joint_publisher)
    joint_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
