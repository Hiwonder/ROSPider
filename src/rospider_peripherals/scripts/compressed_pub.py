#!/usr/bin/env python3

import numpy as np
import cv2
import rospy
from sensor_msgs.msg import CompressedImage, Image

rospy.init_node("image_remap")
pub = rospy.Publisher('/compressed_image', CompressedImage, queue_size=10)

def callback(img):
    rgb_image = np.ndarray(shape=(img.height, img.width, 3), dtype=np.uint8, buffer=img.data) # 原始 RGB 画面
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    msg.data = np.array(cv2.imencode('.jpg', rgb_image)[1]).tostring()
    pub.publish(msg)

rospy.Subscriber('/usb_cam/image_raw', Image, callback, queue_size=10)
rospy.spin()
