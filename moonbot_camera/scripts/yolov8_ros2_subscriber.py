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
        self.inf_LF_subscription = self.create_subscription(
            Yolov8Inference,
            'LFcam/Yolov8_Inference',
            self.LF_yolo_callback,
            10)
        self.inf_LF_subscription

        self.inf_RR_subscription = self.create_subscription(
            Yolov8Inference,
            'RRcam/Yolov8_Inference',
            self.RR_yolo_callback,
            10)
        self.inf_RR_subscription
        ##### Subcriber

        ##### Service Client #####
        self.LF_ConnectionClient = self.create_client(SetBool, 'LF_trigger')
        while not self.LF_ConnectionClient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('LF service not available, waiting again...')
        self.LFreq = SetBool.Request()

        self.RR_ConnectionClient = self.create_client(SetBool, 'RR_trigger')
        while not self.RR_ConnectionClient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('RR service not available, waiting again...')
        self.RRreq = SetBool.Request()
        ##### Service Client #####

        self.RF_class_name = []
        self.LF_class_name = []
        self.LR_class_name = []
        self.RR_class_name = []

        self.leg_number = 4
        self.RF_box = [None] * self.leg_number
        self.LF_box =  [None] * self.leg_number #[top, left, bottom, right]
        self.LR_box = [None] * self.leg_number
        self.RR_box = [None] * self.leg_number

        # self.LF_top = None 
        # self.LF_left = None
        # self.LF_bottom = None
        # self.LF_right = None

        self.detection_class = "connection"
        self.RF_detected = False
        self.RF_countdown = 0

        self.LF_detected = False
        self.LF_countdown = 0
        
        self.LR_detected = False
        self.LR_countdown = 0
        
        self.RR_detected = False
        self.RR_countdown = 0
        
        self.losing_patient = 5 #sec


        # self.img_pub = self.create_publisher(Image, "/inference_result_cv2", 1)

    def LF_send_request(self, request):
        self.LFreq.data = request
        self.future = self.LF_ConnectionClient.call_async(self.LFreq)

    def RR_send_request(self, request):
        self.RRreq.data = request
        self.future = self.RR_ConnectionClient.call_async(self.RRreq)
        

    def LF_yolo_callback(self, data):
        # self.get_logger().info(f"{data.yolov8_inference}")
        for r in data.yolov8_inference:
            self.LF_class_name.append(r.class_name)
            self.LF_box[0] = r.top
            self.LF_box[1] = r.left
            self.LF_box[2] = r.bottom
            self.LF_box[3] = r.right

        # self.get_logger().info(f"{self.class_name}")
        
        detection_class = self.detection_class

        if detection_class in self.LF_class_name and self.LF_detected == False:
            self.LF_detected = True
            self.get_logger().info(f"LF is connected")
            self.LF_send_request(True)

        elif detection_class not in self.LF_class_name and self.LF_detected == True:
            self.LF_countdown += 1
            
            if self.LF_countdown >= self.losing_patient:
                self.LF_detected = False
                self.get_logger().info(f"LF is disconnected")
                self.LF_send_request(False)
                self.LF_countdown = 0
            else:
                self.get_logger().info(f"wait for LF")
                pass

        elif detection_class in self.LF_class_name and self.LF_detected == True:
            self.get_logger().info(f"LF is detected")
            self.LF_countdown = 0

        elif detection_class not in self.LF_class_name and self.LF_detected == False:
            self.get_logger().info(f"No LF detection")

        self.LF_class_name.clear()


    def RR_yolo_callback(self, data):
        # self.get_logger().info(f"{data.yolov8_inference}")
        for r in data.yolov8_inference:
            self.RR_class_name.append(r.class_name)
            self.RR_box[0] = r.top
            self.RR_box[1] = r.left
            self.RR_box[2] = r.bottom
            self.RR_box[3] = r.right

        # self.get_logger().info(f"{self.class_name}")
        
        detection_class = self.detection_class

        if detection_class in self.RR_class_name and self.RR_detected == False:
            self.RR_detected = True
            self.get_logger().info(f"RR is connected")
            self.RR_send_request(True)

        elif detection_class not in self.RR_class_name and self.RR_detected == True:
            self.RR_countdown += 1
            
            if self.RR_countdown >= self.losing_patient:
                self.RR_detected = False
                self.get_logger().info(f"RR is disconnected")
                self.RR_send_request(False)
                self.RR_countdown = 0
            else:
                self.get_logger().info(f"wait for RR")
                pass

        elif detection_class in self.RR_class_name and self.RR_detected == True:
            self.get_logger().info(f"RR is detected")
            self.RR_countdown = 0

        elif detection_class not in self.RR_class_name and self.RR_detected == False:
            self.get_logger().info(f"No RR detection")

        self.RR_class_name.clear()



def main(args=None):
    rclpy.init(args=None)
    yolo_subscriber = Yolo_subscriber()
    rclpy.spin(yolo_subscriber)
    yolo_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

