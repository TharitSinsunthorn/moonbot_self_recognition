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
from dynamixel_hardware.launch import RF, LF, LR, RR, Grieel

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
        super().__init__('LFConnection_service')

        ##### PARAMETERS SETUP ######
        self.declare_parameters(
            namespace='',
            parameters=[
                ('connection_delay', rclpy.Parameter.Type.DOUBLE)
            ]
        )

        self.connection_delay = self.get_parameter(
            "connection_delay").get_parameter_value().double_value
        ##### PARAMETERS SETUP ######

        ##### SERVICE #####
        self.LF_connection_srv = self.create_service(SetBool, 'LF_trigger', self.LF_callback_trigger)
        ##### SERVICE #####

        ##### SetBool #####
        self.LF_ReqBool = SetBool.Request()
        ##### SetBool #####

        ##### LAUNCHER ######
        self.LF_launcher = Ros2LaunchParent()
        self.LF_launch_description = LF.generate_launch_description()
        ##### LAUNCHER ######


    def LF_callback_trigger(self, request, response):
        self.LF_ReqBool = request
        # print(self.LF_ReqBool.data)
        self.get_logger().info(f'Received request: {self.LF_ReqBool.data}')
        
        if self.LF_ReqBool.data == True:
            # self.launch_main(True)
            time.sleep(self.connection_delay)
            self.LF_launcher.start(self.LF_launch_description)

        elif self.LF_ReqBool.data == False:
            # self.launch_main(False)
            time.sleep(self.connection_delay)
            self.LF_launcher.shutdown()

        response.message = "None"
        return response


def main(args=None):
    rclpy.init(args=args)
    
    service = LaunchServiceAsync()

    rclpy.spin(service)

    service.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
