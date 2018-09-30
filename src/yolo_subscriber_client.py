#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import cv2
from std_msgs.msg import Int8
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from image_process.msg import BoundingBoxes,BoundingBox
import numpy as np
from matplotlib import pyplot as plt
from image_process.srv import *
#import scipy as sp
#from scipy import stats

rospy.init_node('img_subscriber', anonymous=True)
r = rospy.Rate(0.5)

def depth_distance_client(Class,xmin,xmax,ymin,ymax):
    rospy.wait_for_service('depth_distance')
    try:
        depth = rospy.ServiceProxy('depth_distance', Boundingbox)
        resp = depth(Class,xmin,xmax,ymin,ymax)
        return resp.distance
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def callback_bounding_boxes(data):
    for i in data.bounding_boxes:
    	if i.Class == "person":
           dist = depth_distance_client(i.Class,i.xmin,i.ymin,i.xmax,i.ymax)
	   print(dist)

def mainloop():
    rospy.wait_for_service('depth_distance')
    rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, callback_bounding_boxes)
    rospy.spin()

if __name__ == '__main__':
    mainloop()
