#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge # Convesion between raw's image msgs and opencv images msgs
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from ultralytics import YOLO
from moonbot_custom_interfaces.msg import InferenceResult
from moonbot_custom_interfaces.msg import Yolov8Inference

bridge = CvBridge()

class PortCamera_subscriber(Node):

	def __init__(self):
		super().__init__('moonbot_detector_node')

		# Define YOLO model
		# model_directory = '/home/tharit/moonbot_ws/src/moonbot_camera/config/runs/detect/yolov8n_v8_50e/weights/best.pt'
		model_directory = '/home/tharit/moonbot_ws/src/moonbot_camera/config/runs/detect/moonbot_perception/weights/best.pt'
		self.model = YOLO(model_directory)

		# Define instance of YOLOv8 inference msg
		self.yolov8_inference_LF = Yolov8Inference()
		self.yolov8_inference_RR = Yolov8Inference()

		# Callbackgroup
		self.group = ReentrantCallbackGroup()
		self.groupLF = MutuallyExclusiveCallbackGroup()
		self.groupRR = MutuallyExclusiveCallbackGroup()

		##### SUBSCRIBER #####
		self.rs_LF_subscription = self.create_subscription(
			Image,
			'LFcam/color/image_raw',
			self.camera_callback_LF,
			10,
			callback_group=self.groupLF)
		self.rs_LF_subscription

		self.rs_RR_subscription = self.create_subscription(
			Image,
			'RRcam/color/image_raw',
			self.camera_callback_RR,
			10,
			callback_group=self.groupRR)
		self.rs_RR_subscription
		##### SUBSCRIBER #####
 
 		##### PUBLISHER #####
		self.rs_LF_yolov8_pub = self.create_publisher(
			Yolov8Inference, 
			"LFcam/Yolov8_Inference", 
			10, 
			callback_group=self.groupLF)
		
		self.rs_LF_img_pub = self.create_publisher(
			Image, 
			"LFcam/color/inference_result", 
			10, 
			callback_group=self.groupLF)

		self.rs_RR_yolov8_pub = self.create_publisher(
			Yolov8Inference, 
			"RRcam/Yolov8_Inference", 
			10, 
			callback_group=self.groupRR)
		
		self.rs_RR_img_pub = self.create_publisher(
			Image, 
			"RRcam/color/inference_result", 
			10, 
			callback_group=self.groupRR)
		##### PUBLISHER #####


	def camera_callback_LF(self, data):

		# Raw's msgs are converted to cv2 image format
		img = bridge.imgmsg_to_cv2(data, "bgr8")
		# Inference is done
		results = self.model.predict(
			source=img, 
			conf=0.85,
			iou=0.2,
			classes=[3], stream=True)

		# results = self.model(source=img, stream=True)

		# Adding framerate and timestamp to a header of YOLOv8 inference msg
		# self.yolov8_inference_LF.header.frame_id = "inference"
		# self.yolov8_inference_LF.header.stamp = PortCamera_subscriber().get_clock().now().to_msg()

		# Putting an array inference results of each object
		for r in results:
			boxes = r.boxes
			for box in boxes:
				self.inference_result = InferenceResult()
				b = box.xyxy[0].to('cpu').detach().numpy().copy() # get box coordinates
				c = box.cls 
				self.inference_result.class_name = self.model.names[int(c)]
				self.inference_result.top  = int(b[0])
				self.inference_result.left = int(b[1])
				self.inference_result.bottom  = int(b[2])
				self.inference_result.right = int(b[3])
				self.yolov8_inference_LF.yolov8_inference.append(self.inference_result)

			# camera_subsciber.get_logger().info(f"{self.yolov8_inference}")
			annotated_frame = r.plot()
		# Extract an annotated img from result and convert it to a raw's msg
		# annotated_frame = results[0].plot()
		img_msg = bridge.cv2_to_imgmsg(annotated_frame)

		# Image and inference results are published
		self.rs_LF_img_pub.publish(img_msg)
		self.rs_LF_yolov8_pub.publish(self.yolov8_inference_LF)
		# Clear an array each time a new topic is received
		self.yolov8_inference_LF.yolov8_inference.clear()

	def camera_callback_RR(self, data):

		# Raw's msgs are converted to cv2 image format
		img = bridge.imgmsg_to_cv2(data, "bgr8")
		# Inference is done
		results = self.model.predict(
			source=img, 
			conf=0.85,
			iou=0.2,
			classes=[3], stream=True)

		# Adding framerate and timestamp to a header of YOLOv8 inference msg
		# self.yolov8_inference_RR.header.frame_id = "inference"
		# self.yolov8_inference_RR.header.stamp = PortCamera_subscriber().get_clock().now().to_msg()

		# Putting an array inference results of each object
		for r in results:
			boxes = r.boxes
			for box in boxes:
				self.inference_result = InferenceResult()
				b = box.xyxy[0].to('cpu').detach().numpy().copy() # get box coordinates
				c = box.cls 
				self.inference_result.class_name = self.model.names[int(c)]
				self.inference_result.top  = int(b[0])
				self.inference_result.left = int(b[1])
				self.inference_result.bottom  = int(b[2])
				self.inference_result.right = int(b[3])
				self.yolov8_inference_RR.yolov8_inference.append(self.inference_result)

			# camera_subsciber.get_logger().info(f"{self.yolov8_inference}")
			annotated_frame = r.plot()
		# Extract an annotated img from result and convert it to a raw's msg
		# annotated_frame = results[0].plot()
		img_msg = bridge.cv2_to_imgmsg(annotated_frame)

		# Image and inference results are published
		self.rs_RR_img_pub.publish(img_msg)
		self.rs_RR_yolov8_pub.publish(self.yolov8_inference_RR)
		# Clear an array each time a new topic is received
		self.yolov8_inference_RR.yolov8_inference.clear()


def main(args=None):
	rclpy.init(args=None)
	try:
		camera_subsciber = PortCamera_subscriber()

		executor = MultiThreadedExecutor()
		executor.add_node(camera_subsciber)

		try:
			executor.spin()
		finally:
			executor.shutdown()
	
	finally:
		# rclpy.spin(camera_subsciber)
		camera_subsciber.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()