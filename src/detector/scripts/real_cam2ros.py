#! /usr/bin/env python3
"""
    Script ...
"""
import pyrealsense2 as rs
import numpy as np
import cv2
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def main():
    rospy.init_node('Realsense_camera', anonymous=True)
    image_pub = rospy.Publisher("image_rgb_topic", Image, queue_size=50)
    bridge = CvBridge()

    # Configure color streams
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The Depth camera haven't a Color sensor")
        exit(0)

    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 15)
    # Start streaming
    pipeline.start(config)

    while not rospy.is_shutdown():
        # Wait for a coherent pair of color frame
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())

        # Converting OpenCV images to ROS image messages
        image_pub.publish(bridge.cv2_to_imgmsg(color_image, "bgr8"))

        # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        # cv2.imshow('RealSense', color_image)
        # cv2.waitKey(1)

    # Stop streaming
    pipeline.stop()

if __name__ == '__main__':
    try:
        main()
    finally:
        print("Shutting down")
