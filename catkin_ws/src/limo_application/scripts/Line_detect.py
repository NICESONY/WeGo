#! /usr/bin/env python3
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int16

import cv2
import numpy as np

class LineDetect:
    def __init__(self):
        self.img_sub_ = rospy.Subscriber('camera/rgb/image_raw/compressed',
                                     CompressedImage,
                                     self.imgCallback)
        self.error_pub_ = rospy.Publisher('gap', Int16, queue_size=10)
        
        self.br_ = CvBridge()
        
        self.YELLOW_LOW_TH = np.array([0, 90, 100])
        self.YELLOW_HIGH_TH = np.array([60, 220, 255])
        
    def imgCallback(self, msg):
        original_image = self.br_.compressed_imgmsg_to_cv2(msg, 'bgr8')
        cv2.imshow('original_image', original_image)
        
        roi_image = original_image[400:480, 0:320]
        cv2.imshow('roi_image', roi_image)
        
        hls = cv2.cvtColor(roi_image, cv2.COLOR_BGR2HLS)
        masked_image = cv2.inRange(hls, self.YELLOW_LOW_TH, self.YELLOW_HIGH_TH)
        cv2.imshow('masked image', masked_image)
        
        M = cv2.moments(masked_image)
        error = Int16()
        
        #M['m00']: 면적, M['m10']: x에 대한 1차 모먼트, M['m01']: y에 대한 1차 모먼트
        if M['m00'] > 20000:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cy += 400
            error.data = 170 -cx
            print(cx, cy)
            print(M['m00'])
            debug_image = cv2.circle(original_image,(cx, cy), 10, (255, 0, 0), -1)
        else:
            error.data = 0
            debug_image = original_image
            
        self.error_pub_.publish(error)   
        cv2.imshow('debug image', debug_image)   
        cv2.waitKey(3)
    
def main():
    rospy.init_node('line_dectect')
    
    dl = LineDetect()
    rospy.spin()

if __name__ == '__main__':
    main()