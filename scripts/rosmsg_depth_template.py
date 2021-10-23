#! /usr/bin/env python3

'''ros utils'''
import rospy
from sensor_msgs.msg import Image, CameraInfo, NavSatFix, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

import cv2
import numpy as np
import time

def cbDepth(msg):
    print("receive depth_altek!")


def CV2msg(cv_image):
    bridge = CvBridge()
    image_message = bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
    return image_message

rospy.init_node("npy_2_rosmsg", anonymous=True)
pubDepth = rospy.Publisher("/camera/depth/image_rect_raw", Image, queue_size=100)
pubDepthCameraInfo = rospy.Publisher("/camera/depth/camera_info", CameraInfo, queue_size=100)

subDepth_altek = rospy.Subscriber("/camera/depth/image_rect_raw", Image, cbDepth)

fDepth = np.load('/home/ncslaber/110-1/211009_allLibrary/front-right/syn_rosbag/depth/30.npy')
msgDepth = CV2msg(fDepth)

msgDepthCameraInfo = CameraInfo()
msgDepthCameraInfo.height = 480
msgDepthCameraInfo.width = 640
msgDepthCameraInfo.distortion_model = "plumb_bob"
msgDepthCameraInfo.header.frame_id = "camera_depth_optical_frame"
msgDepthCameraInfo.D = [0.0, 0.0, 0.0, 0.0, 0.0]
msgDepthCameraInfo.K = [384.31365966796875, 0.0, 320.6562194824219, 0.0, 384.31365966796875, 241.57083129882812, 0.0, 0.0, 1.0]
msgDepthCameraInfo.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
msgDepthCameraInfo.P = [384.31365966796875, 0.0, 320.6562194824219, 0.0, 0.0, 384.31365966796875, 241.57083129882812, 0.0, 0.0, 0.0, 1.0, 0.0]
msgDepthCameraInfo.binning_x = 0
msgDepthCameraInfo.binning_y = 0

rate = rospy.Rate(0.2)
while not rospy.is_shutdown():
    now = rospy.get_rostime()
    
    msgDepth.header.stamp.secs = now.secs
    msgDepth.header.stamp.nsecs = now.nsecs
    pubDepth.publish(msgDepth)

    msgDepthCameraInfo.header.stamp.secs = now.secs
    msgDepthCameraInfo.header.stamp.nsecs = now.nsecs
    pubDepthCameraInfo.publish(msgDepthCameraInfo)

    rate.sleep()

print("successully publish!")
rospy.spin()
