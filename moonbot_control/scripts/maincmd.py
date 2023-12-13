#! /usr/bin/env python3

from cmd_manager_node import CmdManager_ROS 
from moonbot_variables import Body, Leg, Cmds
from body_motion_planner import BodyMotionPlanner
from gait_generator import GaitPlanner
from multiprocessing import Process
import threading
import time
import rclpy
import numpy as np
import math

leg = Leg()
body = Body()
cmd = Cmds()


# lilnk  objects
# leg.foot_zero_pnt[:,2] = body.height  = cmd.body.height
cmd.gait.cycle_time = 0.8
cmd.gait.swing_time = 0.2
cmd.leg.foot_zero_pnt[0,0] = 0.13
cmd.leg.foot_zero_pnt[1,0] = 0.13
cmd.leg.foot_zero_pnt[2,0] = 0.13
cmd.leg.foot_zero_pnt[3,0] = 0.13
cmd.gait.stance_step_h = 0

cmd_manager = CmdManager_ROS(set_msgs=cmd, send_msgs=[leg, body])
gait_planner = GaitPlanner(cmd, leg, body)
bmp = BodyMotionPlanner(cmd, leg, body, gait_planner)

gait_planner.len_zmp_wavegait = 0.050

def main(args=None):
    print('starting')

    thread_cmd_manager = threading.Thread(target=cmd_manager.start)
    thread_gait_planner = threading.Thread(target=gait_planner.run)
    thread_bmp = threading.Thread(target=bmp.run)
    
    # thread_cmd_manager = Process(target=cmd_manager.start)
    # thread_gait_planner = Process(target=gait_planner.run)


    bmp.set_init_pose()

    try:
        thread_cmd_manager.start()
        thread_bmp.start()
        thread_gait_planner.start()
        
        while 1:
            print("tnumber of threads in background: {}".format(threading.active_count()))
            # print("current thread: {}\n".format(threading.current_thread().name))
            # print(cmd.body.height, '----', leg.FR.pose.cur_coord)
            time.sleep(1)
            # print(cmd.mode.start)
            

    except  KeyboardInterrupt:
        cmd_manager.node.destroy_node()
        rclpy.shutdown()
        


if __name__ == '__main__':
    main()