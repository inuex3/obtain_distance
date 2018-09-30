#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import cv2
from std_msgs.msg import Int8
from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from image_process.msg import BoundingBoxes,BoundingBox
import numpy as np
from numpy import inf
from matplotlib import pyplot as plt
from image_process.srv import *
#import scipy as sp
#from scipy import stats
import math

depth_data = Image
bridge = CvBridge()
center_data = np.array
median_data = np.array
inverse_camera_pamrameter = []

#Save the camera parameter to calculate angle and unsubscribe after the first subscription.
def camera_parameter_callback(data):
    global camera_parameter
    camera_parameter_org = data.K
    camera_parameter_org = np.reshape(camera_parameter_org, (3, 3))
    camera_parameter = np.linalg.inv(camera_parameter_org)
    print ("Camera parameter:")
    print (camera_parameter)
    sub_once.unregister()

def imageCallback(data):
    global depth_data
    depth_data = data

def depth_median(req):
    global depth_data
    global center_data
    global median_data
    global camera_parameter
    try:
        cv_image = bridge.imgmsg_to_cv2(depth_data, desired_encoding="passthrough")
    except CvBridgeError as e:
        print(e)
    else:
	width = req.xmax - req.xmin
	height = req.ymax - req.ymin
	center_x = (req.xmax+req.xmin)/2
	center_y = (req.ymax+req.ymin)/2
	cv_image.flags.writeable = True
	cv_image[cv_image == 0.0] = np.nan
	image = cv_image[req.ymin:req.ymax,req.xmin:req.xmax]
	median = np.nanmedian(image)        # 中央値を計算
	angle_x = (camera_parameter[0][0]*center_x+camera_parameter[0][1]*center_y+camera_parameter[0][2]*1)/math.pi*180
	angle_y = (camera_parameter[1][0]*center_x+camera_parameter[1][1]*center_y+camera_parameter[1][2]*1)/math.pi*180
	print(u"dist_median："+str(median)) 
        print(u"angle_x："+str(angle_x))
        print(u"angle_y："+str(angle_y))
    return BoundingboxResponse(median)

def depth_median_server():
    rospy.init_node('depth_distance_server', anonymous=True)
    rospy.Subscriber("/zed/depth/depth_registered", Image, imageCallback)
    global sub_once
    sub_once = rospy.Subscriber("/zed/depth/camera_info", CameraInfo, camera_parameter_callback)
    s = rospy.Service('depth_distance', Boundingbox, depth_median)
    print "Ready to return depth value."
    rospy.spin()

if __name__ == "__main__":
    depth_median_server()
    rospy.spin()
