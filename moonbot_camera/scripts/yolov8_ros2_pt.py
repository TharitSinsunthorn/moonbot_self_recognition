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

class Camera_subscriber(Node):

	def __init__(self):
		super().__init__('moonbot_detector_node')

		# Define YOLO model
		model_directory = '/home/tharit/moonbot_ws/src/moonbot_camera/config/runs/detect/yolov8n_v8_50e/weights/best.pt'
		self.model = YOLO(model_directory)

		# Define instance of YOLOv8 inference msg
		self.yolov8_inference = Yolov8Inference()

		# Callbackgroup
		self.group = ReentrantCallbackGroup()

		self.subscription = self.create_subscription(
			Image,
			'camera/color/image_raw',
			self.camera_callback,
			10,
			callback_group=self.group)
		self.subscription
 
		self.yolov8_pub = self.create_publisher(
			Yolov8Inference, 
			"/Yolov8_Inference", 
			10, 
			callback_group=self.group)
		
		self.img_pub = self.create_publisher(
			Image, 
			"camera/color/inference_result", 
			10, 
			callback_group=self.group)


	def camera_callback(self, data):

		# Raw's msgs are converted to cv2 image format
		img = bridge.imgmsg_to_cv2(data, "bgr8")
		# Inference is done
		results = self.model.predict(
			source=img, 
			conf=0.7,
			iou=0.35,
			classes=[3,4,5])

		# Adding framerate and timestamp to a header of YOLOv8 inference msg
		self.yolov8_inference.header.frame_id = "inference"
		self.yolov8_inference.header.stamp = Camera_subscriber().get_clock().now().to_msg()

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
				self.yolov8_inference.yolov8_inference.append(self.inference_result)

			# camera_subsciber.get_logger().info(f"{self.yolov8_inference}")

		# Extract an annotated img from result and convert it to a raw's msg
		annotated_frame = results[0].plot()
		img_msg = bridge.cv2_to_imgmsg(annotated_frame)

		# Image and inference results are published
		self.img_pub.publish(img_msg)
		self.yolov8_pub.publish(self.yolov8_inference)
		# Clear an array each time a new topic is received
		self.yolov8_inference.yolov8_inference.clear()


def main(args=None):
	rclpy.init(args=None)
	try:
		camera_subsciber = Camera_subscriber()

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