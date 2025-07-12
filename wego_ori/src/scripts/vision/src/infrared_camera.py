#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import cv2
import math
import os
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge , CvBridgeError
import time

import sys                                                                  
import signal
 
def quit(signum, frame):
    sys.exit()


cap = cv2.VideoCapture(1)
rospy.init_node('infrared_camera', anonymous=True) #定义节点
image_pub=rospy.Publisher('/infrared_camera/image_raw', Image, queue_size = 1) #定义话题

signal.signal(signal.SIGINT, quit)                                
signal.signal(signal.SIGTERM, quit)

while True:
    res, img = cap.read()  # BGR
    img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)#将捕获的一帧图像灰度化处理

    ros_frame = Image()
    header = Header(stamp = rospy.Time.now())
    header.frame_id = "infrared_camera"
    ros_frame.header=header
    ros_frame.width = 384
    ros_frame.height = 290
    ros_frame.encoding = "mono8"
    ros_frame.step = 384*1
    ros_frame.data = np.array(img).tostring() #图片格式转换
    image_pub.publish(ros_frame) #发布消息
cap.release()
