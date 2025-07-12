#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
from PIL import Image
import pytesseract
import pyrealsense2 as rs
import numpy as np
import cv2
import rospy
 
# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
 
# Start streaming
pipeline.start(config)
 
try:
    while True:
 
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
 #       depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if  not color_frame:
            continue
 
        # Convert images to numpy arrays
        #depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data()) 
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
 
        # Stack both images horizontally 
        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)        
        x, y = color_image.shape[0:2]
        color_image = cv2.resize(color_image, (int(y / 2), int(x / 2)))
        #cv2.imwrite('test.png',color_image)  # 写入图片
        continue
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        #cv2.imshow('RealSense', gray)
        retVal,binary = cv2.threshold(gray, 100 ,100, cv2.THRESH_BINARY)
        #cv2.imshow('gray_m', binary)
        contours, hierarchy = cv2.findContours(binary,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE) 
        n=len(contours) 
        contoursImg=[]
        for i in range(n):
            if cv2.contourArea(contours[i]) > 80:
                length = cv2.arcLength(contours[i], True)  #获取轮廓长度
                area = cv2.contourArea(contours[i])        #获取轮廓面积
                print('length['+str(i)+']长度=',length)
                print("contours["+str(i)+"]面积=",area)
                temp=np.zeros(binary.shape,np.uint8) #生成黑背景
                contoursImg.append(temp)
                contoursImg[i]=cv2.drawContours(contoursImg[i],contours,i,(255,255,255), 3)  #绘制轮廓
            #cv2.imshow("contours[" + str(i)+"]",contoursImg[i])   #显示轮廓
        #cv2.drawContours(gray_m,contours,-1,(0,0,255),3) 
        #cv2.imshow('contours', binary)
        #text = pytesseract.image_to_string(gray_m)
        #print(text)
 
        key = cv2.waitKey(1)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
 
 
finally:
 
    # Stop streaming
    pipeline.stop()
