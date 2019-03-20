#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import cv2
from std_msgs.msg import Int8
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes,BoundingBox
import numpy as np
from matplotlib import pyplot as plt
#import scipy as sp
#from scipy import stats

rospy.init_node('img_subscriber', anonymous=True)
detected_num = 0
num = 0
frame_num = 0
prev_image_header = ""

def count(data):
    global frame_num
    frame_num = frame_num + 1
    print("frame_num")
    print(frame_num)

def callback_bounding_boxes(data):
    global prev_image_header
    global num
    global frame_num
    detected = False
    if data.image_header != prev_image_header:
        for i in data.bounding_boxes:
            if i.Class == "person":
                detected = True 
        if detected:
            num = num + 1
            print("num")
            print(num)
    prev_image_header = data.image_header

def mainloop():
    rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, callback_bounding_boxes)
    rospy.Subscriber("/zed/left/image_raw_color", Image, count)
    rospy.spin()

if __name__ == '__main__':
    mainloop()
