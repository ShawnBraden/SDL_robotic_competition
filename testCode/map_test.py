import cv2
import numpy as np
import pyrealsense2 as rs

# Initialize RealSense camera
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

# Initialize motion detection parameters
previous_frame = None
motion_threshold = 30  # Adjust according to your environment

# Initialize map
map_size = (480, 640)  # Adjust according to your frame size
map_image = np.zeros((map_size[0], map_size[1], 3), np.uint8)

while True:
    # Get frames from the RealSense camera
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    if not color_frame:
        continue
   
    # Convert RealSense frame to OpenCV format
    frame = np.asanyarray(color_frame.get_data())
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
   
    # Calculate motion vectors
    if previous_frame is not None:
        flow = cv2.calcOpticalFlowFarneback(previous_frame, gray, None, 0.5, 3, 15, 3, 5, 1.2, 0)
        magnitude = np.sqrt(flow[..., 0]**2 + flow[..., 1]**2)
        motion_mask = magnitude > motion_threshold
        # Apply thresholding
        motion_region = np.zeros_like(frame)
        motion_region[motion_mask] = frame[motion_mask]
       
        # Update map
        map_image[motion_mask] = (255, 255, 255)  # Set motion regions to white
        map_image[~motion_mask] = frame[~motion_mask]  # Set non-motion regions to color
       
        # Display map
        cv2.imshow('Map', map_image)
   
    # Update previous frame
    previous_frame = gray.copy()
   
    # Exit on ESC
    if cv2.waitKey(1) == 27:
        break

# Release resources
pipeline.stop()
cv2.destroyAllWindows()