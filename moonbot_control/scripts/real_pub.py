#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from rclpy.duration import Duration
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
# from moonbot_custom_interfaces.msg import Detection

from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor


import numpy as np
import math
from IK.limb_kinematics import InvKinematics

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation


class JointPublisher(Node):

    def __init__(self):
        super().__init__('autoseq_gait_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('gait_type', 'crawl'),
                ('speed', 5),
                ('use_plot_debug', False)
            ]
        )

        self.gait_type = self.get_parameter('gait_type').get_parameter_value().string_value
        self.speed = self.get_parameter('speed').get_parameter_value().integer_value
        self.use_plot_debug = self.get_parameter('use_plot_debug').get_parameter_value().bool_value

        self.group = ReentrantCallbackGroup()
        self.timer_group = ReentrantCallbackGroup()

        ##### PUBLISHER ######
        self.RF_joint_publisher = self.create_publisher(
            JointTrajectory,
            '/RF/position_trajectory_controller/joint_trajectory',
            10, 
            callback_group=self.group)

        self.LF_joint_publisher = self.create_publisher(
            JointTrajectory,
            '/LF/position_trajectory_controller/joint_trajectory',
            10,
            callback_group=self.group)

        self.LR_joint_publisher = self.create_publisher(
            JointTrajectory,
            '/LR/position_trajectory_controller/joint_trajectory',
            10,
            callback_group=self.group)

        self.RR_joint_publisher = self.create_publisher(
            JointTrajectory,
            '/RR/position_trajectory_controller/joint_trajectory',
            10,
            callback_group=self.group)
        ##### PUBLISHER #####


        ##### TIMER ######
        self.timer_period = self.speed
        self.RF_timer = self.create_timer(self.timer_period, self.pub_callback)
        ##### TIMER ######

        self.IK = InvKinematics()

        ##### Gait Parameter #####
        self.start_walk = False
        self.loop_shift = np.zeros([4,3])
        self.swing_sample = 20
        self.stance_sample = 20
        self.zmp_sample = 10
        self.step_len = 0.04
        self.step_goal = np.array([0.0, self.step_len, 0.0])
        self.stance_goal = np.array([0.0, -self.step_len, 0.0])
        self.stance_init = np.array([0.0, 0.0, 0.0])
        self.step_height = 0.05
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

        self.zero_pnt = np.array([[self.span, self.span, self.height], 
                                  [-self.span, self.span, self.height],
                                  [-self.span, -self.span, self.height], 
                                  [self.span, -self.span, self.height]])

        ##### Pose Parameters #####

        self.ang_RF = []
        self.ang_LF = []
        self.ang_LR = []
        self.ang_RR = []
        self.all_joint_angles = []


    def zmp_handler(self, shift, sample=10):
        start_point = [self.RF_pose[-1] - self.zero_pnt[0], 
                       self.LF_pose[-1] - self.zero_pnt[1], 
                       self.LR_pose[-1] - self.zero_pnt[2], 
                       self.RR_pose[-1] - self.zero_pnt[3]]

        goal = -np.array([shift]*4) + np.array([[0,start_point[0][1],0], [0,start_point[1][1],0], [0,start_point[2][1],0], [0,start_point[3][1],0]])

        for i in range(sample):
            RF_zmp = (goal[0] - start_point[0]) * (i+1)/sample #* np.array([1,1,0])
            LF_zmp = (goal[1] - start_point[1]) * (i+1)/sample #* np.array([1,1,0])
            LR_zmp = (goal[2] - start_point[2]) * (i+1)/sample #* np.array([1,1,0])
            RR_zmp = (goal[3] - start_point[3]) * (i+1)/sample #* np.array([1,1,0])

            self.RF_pose.append(RF_zmp + start_point[0] + self.zero_pnt[0])
            self.LF_pose.append(LF_zmp + start_point[1] + self.zero_pnt[1])
            self.LR_pose.append(LR_zmp + start_point[2] + self.zero_pnt[2])
            self.RR_pose.append(RR_zmp + start_point[3] + self.zero_pnt[3])
        # self.stance_RF(np.array(shift))
        # self.stance_LF(np.array(shift))
        # self.stance_LR(np.array(shift))
        # self.stance_RR(np.array(shift))





    def swing_RF(self, goal):
        trajRF = []
        start_point = np.array(self.RF_pose[-1]) - self.zero_pnt[0]
        stride = goal - start_point
        div = (stride[1]) / self.swing_sample

        for j in range(0, self.swing_sample):
            x = 0.0
            y = div*(j+1)
            z = - self.step_height*np.sin(math.pi/stride[1] * y)

            trajRF = np.array(start_point) + np.array([x,y,z])
            self.RF_pose.append(trajRF + self.zero_pnt[0])

    def stance_RF(self, goal):
        trajRF = []
        start_point = np.array(self.RF_pose[-1]) - self.zero_pnt[0]
        stride = goal - start_point
        div = (stride) / self.swing_sample

        for j in range(0,self.stance_sample):
            x = 0.0
            y = div[1]*(j+1)
            z = 0.0

            trajRF = np.array(start_point) + np.array([x,y,z])
            self.RF_pose.append(trajRF + self.zero_pnt[0])

    def swing_LF(self, goal):
        trajLF = []
        start_point = np.array(self.LF_pose[-1]) - self.zero_pnt[1]
        stride = goal - start_point
        div = (stride[1]) / self.swing_sample

        for j in range(0, self.swing_sample):
            x = 0.0
            y = div*(j+1)
            z = - self.step_height*np.sin(math.pi/stride[1] * y)

            trajLF = np.array(start_point) + np.array([x,y,z])
            self.LF_pose.append(trajLF + self.zero_pnt[1])

    def stance_LF(self, goal):
        trajLF = []
        start_point = np.array(self.LF_pose[-1]) - self.zero_pnt[1]
        stride = goal - start_point
        div = (stride) / self.swing_sample   

        for j in range(0,self.stance_sample):
            x = 0.0
            y = div[1]*(j+1)
            z = 0.0

            trajLF = np.array(start_point) + np.array([x,y,z])
            self.LF_pose.append(trajLF + self.zero_pnt[1])

    def swing_LR(self, goal):
        trajLR = []
        start_point = np.array(self.LR_pose[-1]) - self.zero_pnt[2]
        stride = goal - start_point
        div = (stride[1]) / self.swing_sample

        for j in range(0,self.swing_sample):
            x = 0.0
            y = div*(j+1)
            z = - self.step_height*np.sin(math.pi/stride[1] * y)

            trajLR = np.array(start_point) + np.array([x,y,z])
            self.LR_pose.append(trajLR + self.zero_pnt[2])

    def stance_LR(self, goal):
        trajLR = []
        start_point = np.array(self.LR_pose[-1]) - self.zero_pnt[2]
        stride = goal - start_point
        div = (stride) / self.swing_sample
    
        for j in range(0,self.stance_sample):
            x = 0.0
            y = div[1]*(j+1)
            z = 0.0

            trajLR = np.array(start_point) + np.array([x,y,z])
            self.LR_pose.append(trajLR + self.zero_pnt[2])

    def swing_RR(self, goal):
        trajRR = []
        start_point = np.array(self.RR_pose[-1]) - self.zero_pnt[3]
        
        stride = goal-start_point
        div = (stride[1]) / self.swing_sample

        for j in range(0, self.swing_sample):
            x = 0.0
            y = div*(j+1)
            z = - self.step_height*np.sin(math.pi/stride[1] * y)

            trajRR = np.array(start_point) + np.array([x,y,z])
            self.RR_pose.append(trajRR + self.zero_pnt[3])

    def stance_RR(self, goal):
        trajRR = []
        start_point = np.array(self.RR_pose[-1]) - self.zero_pnt[3]
        stride = goal-start_point
        div = (stride) / self.swing_sample

        for j in range(0,self.stance_sample):
            x = 0.0
            y = div[1]*(j+1)
            z = 0.0

            trajRR = np.array(start_point) + np.array([x,y,z])
            self.RR_pose.append(trajRR + self.zero_pnt[3])

    def trot_gait(self):

        self.sample_time = self.timer_period / (2*self.swing_sample)

        self.stance_RF(self.stance_goal)
        self.swing_LF(self.step_goal)
        self.stance_LR(self.stance_goal)
        self.swing_RR(self.step_goal)

        self.swing_RF(self.step_goal)
        self.stance_LF(self.stance_init)
        self.swing_LR(self.step_goal)
        self.stance_RR(self.stance_init)

        self.start_walk == True

        # self.plot_debug()


    def trot_gait2(self):

        self.sample_time = self.timer_period / (4*self.swing_sample)

        self.stance_RF(self.stance_init)
        self.swing_LF(self.step_goal)
        self.stance_LR(self.stance_init)
        self.swing_RR(self.step_goal)

        self.stance_RF(self.stance_goal)
        self.stance_LF(self.stance_init)
        self.stance_LR(self.stance_goal)
        self.stance_RR(self.stance_init)

        self.swing_RF(self.step_goal)
        self.stance_LF(self.stance_init)
        self.swing_LR(self.step_goal)
        self.stance_RR(self.stance_init)

        self.stance_RF(self.stance_init)
        self.stance_LF(self.stance_goal)
        self.stance_LR(self.stance_init)
        self.stance_RR(self.stance_goal) 

        self.start_walk == True

        # self.plot_debug()

    def crawl_gait(self):

        self.sample_time = self.timer_period / (4*self.zmp_sample + 4*self.swing_sample)

        if self.start_walk == True:
            self.loop_shift = [self.stance_goal, -self.stance_goal/3, self.stance_goal/3, np.zeros(3)] 
            # self.loop_shift = self.loop_shift
            # RF_loop_stance = [-s]

        self.zmp_handler([-0.035, 0.0, 0.0], 2*self.zmp_sample)

        self.stance_RF(self.stance_init + self.loop_shift[0])
        self.stance_LF(self.stance_init + self.loop_shift[1])
        self.stance_LR(self.stance_init + self.loop_shift[2])
        self.swing_RR(self.step_goal)

        self.swing_RF(self.step_goal)
        self.stance_LF(2*self.stance_goal/3 + self.loop_shift[1])
        self.stance_LR(2*self.stance_goal/3 + self.loop_shift[2])
        self.stance_RR(-self.stance_goal/3)

        self.zmp_handler([0.035, 0.0, 0.0], 2*self.zmp_sample)

        self.stance_RF(-self.stance_goal/3)
        self.stance_LF(4*self.stance_goal/3 + self.loop_shift[1])
        self.swing_LR(self.step_goal)
        self.stance_RR(self.stance_goal/3)

        self.stance_RF(self.stance_goal/3)
        self.swing_LF(self.step_goal)
        self.stance_LR(-self.stance_goal/3)
        self.stance_RR(self.stance_goal)

        self.start_walk = True

        # self.plot_debug()
        # self.plot_debug_animation()

    
    def pub_callback(self):

        if self.gait_type == 'crawl' or self.gait_type == 'Crawl':
            self.crawl_gait()
        
        elif self.gait_type == 'trot' or self.gait_type == 'Trot':
            self.trot_gait()

        elif self.gait_type == 'trot2' or self.gait_type == 'Trot2':
            self.trot_gait2()

        else:
            self.get_logger().info(f'The gait is not in service :(')

        for i in range(len(self.RF_pose)):
            self.ang_RF.append(self.IK.get_RF_joint_angles(self.RF_pose[i], [0,0,0]))
            self.ang_LF.append(self.IK.get_LF_joint_angles(self.LF_pose[i], [0,0,0]))
            self.ang_LR.append(self.IK.get_LR_joint_angles(self.LR_pose[i], [0,0,0]))
            self.ang_RR.append(self.IK.get_RR_joint_angles(self.RR_pose[i], [0,0,0]))

        self.RobotPub()

        self.get_logger().info(f'pub')

        self.cleardata()


 
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

        sec = self.sample_time

        RF_points = []
        LF_points = []
        LR_points = []
        RR_points = []
        for i in range(len(RF)):
            RF_point = JointTrajectoryPoint()
            LF_point = JointTrajectoryPoint()
            LR_point = JointTrajectoryPoint()
            RR_point = JointTrajectoryPoint()

            RF_point.time_from_start = Duration(seconds=(i+1)*sec + 0, nanoseconds=0).to_msg()
            LF_point.time_from_start = Duration(seconds=(i+1)*sec + 0, nanoseconds=0).to_msg()
            LR_point.time_from_start = Duration(seconds=(i+1)*sec + 0, nanoseconds=0).to_msg()
            RR_point.time_from_start = Duration(seconds=(i+1)*sec + 0, nanoseconds=0).to_msg()

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


    def plot_debug(self):
        # Extract coordinates for plotting
        d = 0.0
        offset =  np.array([[d,d,0.0], [-d,d,0.0], [-d,-d,0.0], [d,-d,0.0]])

        RF_pose = (self.RF_pose + np.ones([len(self.RF_pose),3])*offset[0,:]) * np.array([[1,1,-1]])
        LF_pose = (self.LF_pose + np.ones([len(self.LF_pose),3])*offset[1,:]) * np.array([[1,1,-1]])
        LR_pose = (self.LR_pose + np.ones([len(self.LR_pose),3])*offset[2,:]) * np.array([[1,1,-1]])
        RR_pose = (self.RR_pose + np.ones([len(self.RR_pose),3])*offset[3,:]) * np.array([[1,1,-1]])

        # Extract coordinates for plotting
        RF_X, RF_Y, RF_Z = zip(*RF_pose)
        LF_X, LF_Y, LF_Z = zip(*LF_pose)
        LR_X, LR_Y, LR_Z = zip(*LR_pose)
        RR_X, RR_Y, RR_Z = zip(*RR_pose)

        # Plotting
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Plot points for each pose
        ax.scatter(RF_X, RF_Y, RF_Z, c='r', marker='o', label='RF Pose')
        ax.scatter(LF_X, LF_Y, LF_Z, c='g', marker='^', label='LF Pose')
        ax.scatter(LR_X, LR_Y, LR_Z, c='b', marker='s', label='LR Pose')
        ax.scatter(RR_X, RR_Y, RR_Z, c='m', marker='x', label='RR Pose')

        #ax.set_xlim([0.2, 0.3])
        # ax.set_ylim([-0.3, 0.3])
        ax.set_zlim([-self.height, -self.height + 0.5])

        # Set labels and title
        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')
        ax.set_zlabel('Z-axis')
        ax.set_title('3D Plot of Poses')

        
        # Add a legend
        ax.legend()

        plt.show()

    def cleardata(self):
        # self.RF_pose = [[self.span, self.span, self.height]]
        # self.LF_pose = [[-self.span, self.span, self.height]]
        # self.LR_pose = [[-self.span, -self.span, self.height]]
        # self.RR_pose = [[self.span, -self.span, self.height]]

        last_pose = [self.RF_pose[-1], self.LF_pose[-1], self.LR_pose[-1], self.RR_pose[-1]]

        self.RF_pose = [last_pose[0]]
        self.LF_pose = [last_pose[1]]
        self.LR_pose = [last_pose[2]]
        self.RR_pose = [last_pose[3]]

        self.ang_RF.clear()
        self.ang_LF.clear()
        self.ang_LR.clear()
        self.ang_RR.clear()

        self.get_logger().info(f'{last_pose}')



def main(args=None):
    rclpy.init(args=args)

    joint_publisher = JointPublisher()

    rclpy.spin(joint_publisher)

    joint_publisher.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
