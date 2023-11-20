#!/usr/bin/env python3

import cv2
import threading 
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool

import time
from cv_bridge import CvBridge
from moonbot_custom_interfaces.msg import Yolov8Inference

bridge = CvBridge()

class Yolo_subscriber(Node):

    def __init__(self):
        super().__init__('yolo_subscriber')

        ##### Subcriber ##### 
        self.subscription = self.create_subscription(
            Yolov8Inference,
            '/Yolov8_Inference',
            self.yolo_callback,
            10)
        self.subscription
        ##### Subcriber

        ##### Service Client #####
        self.Leg_ConnectionClient = self.create_client(SetBool, 'leg_trigger')
        while not self.Leg_ConnectionClient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetBool.Request()
        ##### Service Client #####

        # self.timer = self.create_timer(1.0, self.timer_callback)

        self.class_name = []
        self.top = None 
        self.left = None
        self.bottom = None
        self.right = None

        self.last_leg_detected = 0.0
        self.leg_detected = False


        # self.img_pub = self.create_publisher(Image, "/inference_result_cv2", 1)

    def send_request(self, request):
        self.req.data = request
        self.future = self.Leg_ConnectionClient.call_async(self.req)
        

    def yolo_callback(self, data):
        # global img
        # self.get_logger().info(f"{data.yolov8_inference}")

        for r in data.yolov8_inference:

            self.class_name.append(r.class_name)
            self.top = r.top
            self.left = r.left
            self.bottom = r.bottom
            self.right = r.right

        # self.get_logger().info(f"{self.class_name}")
        

        if "connection" in self.class_name and self.leg_detected == False:
        #   time.sleep(3)
            self.leg_detected = True
            self.last_leg_detected == time.time()
            self.get_logger().info(f"leg is connected")
            self.send_request(True)

        elif "connection" not in self.class_name and self.leg_detected == True:
            self.leg_detected = False
            self.get_logger().info(f"leg is disconnected")
            self.send_request(False)

        
        elif "connection" in self.class_name and self.leg_detected == True:
            self.get_logger().info(f"leg is detected")

        elif "connection" not in self.class_name and self.leg_detected == False:
            self.get_logger().info(f"No leg detected")

        self.class_name.clear()


    def timer_callback(self):
        # Your periodic processing logic here
        pass

def main(args=None):
    rclpy.init(args=None)
    yolo_subscriber = Yolo_subscriber()
    # camera_subscriber = Camera_subscriber()

    # executor = MultiThreadedExecutor()
    # executor.add_node(yolo_subscriber)
    # executor.add_node(camera_subscriber)

    # executor_thread = threading.Thread(target=executor.spin, daemon=True)
    # executor_thread.start()

    # rate = yolo_subscriber.create_rate(2)
    # try:
    #   while rclpy.ok():
    #       rate.sleep()
    # except KeyboardInterrupt:
    #   pass 

    rclpy.spin(yolo_subscriber)
    rclpy.shutdown()
    # executor_thread.join

if __name__ == '__main__':
    main()

