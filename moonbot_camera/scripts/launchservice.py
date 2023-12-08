#!/usr/bin/env python3

import sys
import time
import asyncio
import multiprocessing

from launch import LaunchService
# from yolo import generate_launch_description
sys.path.append('../moonbot_ws/src')
# from moonbot_camera.launch import *
# from dynamixel_hardware.launch.Grieel import generate_launch_description
from dynamixel_hardware.launch import *

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from std_srvs.srv import SetBool, Trigger


class Ros2LaunchParent:
    def start(self, launch_description):
        self._stop_event = multiprocessing.Event()
        self._process = multiprocessing.Process(target=self._run_process, args=(self._stop_event, launch_description), daemon=True)
        self._process.start()

    def shutdown(self):
        self._stop_event.set()
        self._process.join()

    def _run_process(self, stop_event, launch_description):
        loop = asyncio.get_event_loop()
        launch_service = LaunchService()
        launch_service.include_launch_description(launch_description)
        launch_task = loop.create_task(launch_service.run_async())
        loop.run_until_complete(loop.run_in_executor(None, stop_event.wait))
        if not launch_task.done():
            asyncio.ensure_future(launch_service.shutdown(), loop=loop)
            loop.run_until_complete(launch_task)

class LaunchServiceAsync(Node):

    def __init__(self):
        super().__init__('LegConnection_service')

        ##### Callbackgroup #####
        self.group = ReentrantCallbackGroup()
        ##### Callbackgroup #####

        ##### SERVICE #####
        self.RF_connection_srv = self.create_service(SetBool, 'RF_trigger', self.RF_callback_trigger, callback_group=self.group)
        self.LF_connection_srv = self.create_service(SetBool, 'LF_trigger', self.LF_callback_trigger, callback_group=self.group)
        self.LR_connection_srv = self.create_service(SetBool, 'LR_trigger', self.LR_callback_trigger, callback_group=self.group)
        self.RR_connection_srv = self.create_service(SetBool, 'RR_trigger', self.RR_callback_trigger, callback_group=self.group)
        
        ##### SERVICE #####

        ##### SetBool #####
        self.RF_ReqBool = SetBool.Request()
        self.LF_ReqBool = SetBool.Request()
        self.LR_ReqBool = SetBool.Request()
        self.RR_ReqBool = SetBool.Request()
        ##### SetBool #####

        ##### LAUNCHER ######
        self.RF_launcher = Ros2LaunchParent()
        self.RF_launch_description = RF.generate_launch_description()

        self.LF_launcher = Ros2LaunchParent()
        self.LF_launch_description = LF.generate_launch_description()

        self.LR_launcher = Ros2LaunchParent()
        self.LR_launch_description = LR.generate_launch_description()

        self.RR_launcher = Ros2LaunchParent()
        self.RR_launch_description = RR.generate_launch_description()
        ##### LAUNCHER ######


    def RF_callback_trigger(self, request, response):
        self.RF_ReqBool = request
        # print(self.LF_ReqBool.data)
        self.get_logger().info(f'Received request: {self.RF_ReqBool.data}')
        
        if self.RF_ReqBool.data == True:
            # self.launch_main(True)
            time.sleep(2)
            self.RF_launcher.start(self.RF_launch_description)

        elif self.RF_ReqBool.data == False:
            # self.launch_main(False)
            time.sleep(2)
            self.RF_launcher.shutdown()

        response.message = "None"
        return response


    def LF_callback_trigger(self, request, response):
        self.LF_ReqBool = request
        # print(self.LF_ReqBool.data)
        self.get_logger().info(f'Received request: {self.LF_ReqBool.data}')
        
        if self.LF_ReqBool.data == True:
            # self.launch_main(True)
            time.sleep(2)
            self.LF_launcher.start(self.LF_launch_description)

        elif self.LF_ReqBool.data == False:
            # self.launch_main(False)
            time.sleep(2)
            self.LF_launcher.shutdown()

        response.message = "None"
        return response


    def LR_callback_trigger(self, request, response):
        self.LR_ReqBool = request
        # print(self.LF_ReqBool.data)
        self.get_logger().info(f'Received request: {self.LR_ReqBool.data}')
        
        if self.LR_ReqBool.data == True:
            # self.launch_main(True)
            time.sleep(2)
            self.LR_launcher.start(self.LR_launch_description)

        elif self.LR_ReqBool.data == False:
            # self.launch_main(False)
            time.sleep(2)
            self.LR_launcher.shutdown()

        response.message = "None"
        return response


    def RR_callback_trigger(self, request, response):
        self.RR_ReqBool = request
        # print(self.LF_ReqBool.data)
        self.get_logger().info(f'Received request: {self.RR_ReqBool.data}')
        
        if self.RR_ReqBool.data == True:
            # self.launch_main(True)
            time.sleep(2)
            self.RR_launcher.start(self.RR_launch_description)

        elif self.RR_ReqBool.data == False:
            # self.launch_main(False)
            time.sleep(2)
            self.RR_launcher.shutdown()

        response.message = "None"
        return response
    

def main(args=None):
    rclpy.init(args=args)
    
    # try:
    #     service_node = LaunchServiceAsync()

    #     executor = MultiThreadedExecutor()
    #     executor.add_node(service_node)
    
    #     try:
    #         executor.spin()
    #     finally:
    #         executor.shutdown()
    
    # finally:        
    #     service_node.destroy_node()
    #     rclpy.shutdown()
    service = LaunchServiceAsync()

    rclpy.spin(service)

    sevice.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
