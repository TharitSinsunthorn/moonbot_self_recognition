# credit https://www.youtube.com/watch?v=xqroBkpf3lY&t=368s

import cv2 
import sys 
import os 
import time 
import numpy as np   
import pyrealsense2 as rs
from ultralytics import YOLO   

W = 640
H = 480

# Define format and framerate
config = rs.config()
config.enable_stream(rs.stream.color, W, H, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, W, H, rs.format.z16, 30)

# Start pipeline and stream the above format
pipeline = rs.pipeline()
profile = pipeline.start(config)

# The alignment utility performs per pixel geometric transformation based on the depth data
align_to = rs.stream.color   
align = rs.align(align_to)

# Define YOLO model
model_directory = '/home/tharit/moonbot_ws/src/moonbot_camera/config/runs/detect/yolov8n_v8_50e/weights/best.pt'
# model_directory = '/home/tharit/realsense_ws/src/yolotraining/yolov8n.pt'
model = YOLO(model_directory)


while True:
    # Wait until the frame become available
    frames = pipeline.wait_for_frames()

    # Get align fram from rgb and depth camera
    aligned_frames = align.process(frames)
    color_frame = aligned_frames.get_color_frame()
    depth_frame = aligned_frames.get_depth_frame()
    if not color_frame:
        continue

    # Convert depth image to color image to add colored bounding boxes later
    color_image = np.asanyarray(color_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data())
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.08), cv2.COLORMAP_JET)

    # Inference is done here
    results = model.predict(
            source=color_image, 
            conf=0.5,
            iou=0.75)

    for r in results: 
        boxes = r.boxes
        print(r.keypoints)
        for box in boxes:
            b = box.xyxy[0].cpu().detach().numpy().copy() # get box coordinates in (top, left, bottom, right) format
            c = box.cls                                       # Obtain index of detected class

            # Draw a rectangle on a depth color map and put the text on the rectangle
            # For xyxy
            cv2.rectangle(depth_colormap, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), (0, 0, 255),
                            thickness = 2, lineType=cv2.LINE_4)
            # For xywh
            # cv2.rectangle(depth_colormap, (int(b[0]-b[2]/2), int(b[1]+b[3]/2)), (int(b[0]+b[2]/2), int(b[1]-b[3]/2)), (0, 0, 255),
            #                 thickness = 2, lineType=cv2.LINE_4)

            cv2.rectangle(depth_colormap, (0, 0), (200, 200), (0, 255, 255),
                            thickness = 2, lineType=cv2.LINE_4)

            cv2.putText(depth_colormap, text = model.names[int(c)], org=(int(b[0]), int(b[1])),
                        fontFace = cv2.FONT_HERSHEY_SIMPLEX, fontScale = 0.7, color = (0, 0, 255),
                        thickness = 2, lineType=cv2.LINE_4)

    # Obtain inference result image from YOLO
    annotated_frame = results[0].plot()


    depth_colormap_dim = depth_colormap.shape
    color_colormap_dim = annotated_frame.shape

    # If depth and color resolutions are different, resize color image to match depth image for display
    if depth_colormap_dim != color_colormap_dim:
        resized_color_image = cv2.resize(annotated_frame, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
        images = np.hstack((resized_color_image, depth_colormap))
    else:
        images = np.hstack((annotated_frame, depth_colormap))

    # Show images
    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('RealSense', images)
        
    # Create window with result
    # cv2.imshow("color_image", annotated_frame)
    # cv2.imshow("depth_image", depth_colormap)

    # time2 = time.time()
    # print(f"FPS : {int(1/(time2 - time1))}")

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
