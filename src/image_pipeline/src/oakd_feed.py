#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Configuration for depth processing
extended_disparity = False
subpixel = False
lr_check = True

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
depth = pipeline.create(dai.node.StereoDepth)
xout = pipeline.create(dai.node.XLinkOut)

xout.setStreamName("disparity")

# Properties
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setFps(24)  # Lower frame rate
monoRight.setFps(24)

# Depth map configuration
depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
depth.setLeftRightCheck(lr_check)
depth.setExtendedDisparity(extended_disparity)
depth.setSubpixel(subpixel)

# Post-processing filters, optimized
config = depth.initialConfig.get()
config.postProcessing.speckleFilter.enable = False
config.postProcessing.temporalFilter.enable = False  # Disable temporal filter
config.postProcessing.spatialFilter.enable = True
config.postProcessing.spatialFilter.holeFillingRadius = 1  # Reduced radius
config.postProcessing.thresholdFilter.minRange = 400
config.postProcessing.thresholdFilter.maxRange = 15000
depth.initialConfig.set(config)

# Linking
monoLeft.out.link(depth.left)
monoRight.out.link(depth.right)
depth.disparity.link(xout.input)

# Initialize ROS node
rospy.init_node("oakd_disparity_publisher", anonymous=True)
pub = rospy.Publisher("oakd_disparity", Image, queue_size=10)
bridge = CvBridge()

# Connect to device and start pipeline
with dai.Device(pipeline) as device:

    # Output queue for disparity frames
    q = device.getOutputQueue(name="disparity", maxSize=8, blocking=False)

    while not rospy.is_shutdown():
        inDisparity = q.get()
        frame = inDisparity.getFrame()

        # Normalize disparity map for better visualization
        frame = (frame * (255 / depth.initialConfig.getMaxDisparity())).astype(np.uint8)

        # Convert to color map (optional, but can be useful for visualization)
        frame_color = cv2.applyColorMap(frame, cv2.COLORMAP_JET)

        # Publish both grayscale and color frames
        pub.publish(bridge.cv2_to_imgmsg(frame, "mono8"))
        # pub.publish(bridge.cv2_to_imgmsg(frame_color, "bgr8"))

        rospy.Rate(10).sleep()  # Adjust the rate as needed
