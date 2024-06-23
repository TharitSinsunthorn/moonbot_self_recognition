#! /usr/bin/env python3

from cmd_manager_node import CmdManager_ROS 
from moonbot_variables import Body, Leg, Cmds
from body_motion_planner import BodyMotionPlanner
from gait_generator import GaitPlanner
import threading
import time
import rclpy

leg = Leg()
body = Body()
cmd = Cmds()

cmd.gait.stance_step_h = 0

cmd_manager = CmdManager_ROS(set_msgs=cmd, send_msgs=[leg, body])
gait_planner = GaitPlanner(cmd, leg, body)
bmp = BodyMotionPlanner(cmd, leg, body, gait_planner)

def main(args=None):
    '''
    Main executor for running each program parallely
    '''
    print('starting')

    thread_cmd_manager = threading.Thread(target=cmd_manager.start)
    thread_gait_planner = threading.Thread(target=gait_planner.run)
    thread_bmp = threading.Thread(target=bmp.run)
    
    # bmp.set_init_pose()

    try:
        thread_cmd_manager.start()
        thread_bmp.start()
        thread_gait_planner.start()
        
        while 1:
            print("tnumber of threads in background: {}".format(threading.active_count()))
            print("current thread: {}\n".format(threading.current_thread().name))
            time.sleep(1)
            

    except  KeyboardInterrupt:
        cmd_manager.node.destroy_node()
        rclpy.shutdown()
        


if __name__ == '__main__':
    main()