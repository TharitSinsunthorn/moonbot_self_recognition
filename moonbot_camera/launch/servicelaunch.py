import time
import asyncio
import multiprocessing

from launch import LaunchService
# from yolo import generate_launch_description
from test import generate_launch_description

import rclpy
from rclpy.node import Node
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
        self.srv = self.create_service(SetBool, 'leg_trigger', self.callback_trigger)
        self.ReqBool = SetBool.Request()

        self.launcher = Ros2LaunchParent()
        self.launch_description = generate_launch_description()

    def callback_trigger(self, request, response):
        self.ReqBool = request
        # print(self.ReqBool.data)
        self.get_logger().info(f'Received request: {self.ReqBool.data}')
        
        if self.ReqBool.data == True:
            # self.launch_main(True)
            time.sleep(3)
            self.launcher.start(self.launch_description)

        elif self.ReqBool.data == False:
            # self.launch_main(False)
            time.sleep(3)
            self.launcher.shutdown()

        response.message = "None"
        return response



    # def launch_main(self, trigger):
    #     launcher = Ros2LaunchParent()
    #     launch_description = generate_launch_description()

    #     if trigger == True:
    #         time.sleep(5)
    #         launcher.start(launch_description)
    #         # time.sleep(5)
    #         # launcher.shutdown()

    #         # Sleep indefinitely, or until the service receives a request with data=False
    #         # i=0
    #         while True:
    #             print("running")
    #             # i+=1
    #             time.sleep(1)
    #             if self.ReqBool.data == "False":
    #                 break

    #         # Shutdown the launcher when data=False is received
    #         launcher.shutdown()
    
def main(args=None):
    rclpy.init(args=args)
    service = LaunchServiceAsync()

    rclpy.spin(service)
    
    sevice.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
