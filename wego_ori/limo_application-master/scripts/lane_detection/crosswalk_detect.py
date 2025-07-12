#! /usr/bin/env python
# -*- coding: utf-8 -*-
# limo_application/scripts/lane_detection/crosswalk_detect.py
# WeGo LIMO Pro를 이용한 횡단보도 인식 코드

import rospy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
from dynamic_reconfigure.server import Server
from limo_application.cfg import crosswalkConfig

import cv2
import numpy as np


class CrossWalkDetector:
    '''
        ROS 기반 횡단보도 인식 객체
        Private Params --> image_topic_name, visualization
        Image Topic Subscriber (CompressedImage Type)
        Distance to Stop Line Topic Publisher (Int32 Type)
    '''
    def __init__(self):
        # ROS Part
        rospy.init_node("crosswalk_detect")
        srv = Server(crosswalkConfig, self.reconfigure_callback)
        self.cvbridge = CvBridge()
        rospy.Subscriber(rospy.get_param("~image_topic_name", "/camera/rgb/image_raw/compressed"), CompressedImage, self.Image_CB)
        self.distance_pub = rospy.Publisher("/limo/crosswalk_y", Int32, queue_size=5)
        self.viz = rospy.get_param("~visualization", False)

    # return Int32
    def calcCrossWalkDistance(self, _img):
        '''
            최종 검출된 Binary 이미지를 이용하여, 횡단 보도의 모멘트 계산
            모멘트의 x, y 좌표 중 차량과의 거리에 해당하는 y를 반환
        '''
        if self.line_num >= self.CROSS_WALK_DETECT_TH:
            try:
                M = cv2.moments(_img)
                self.x = int(M['m10']/M['m00'])
                self.y = int(M['m01']/M['m00'])
            except:
                self.x = -1
                self.y = -1
            # print("x, y = {}, {}".format(x, y))
            return self.y
        else:
            self.x = 0
            self.y = 0
            return -1

    def visResult(self):
        '''
            최종 결과가 추가된 원본 이미지 (crosswalk_original)
            횡단 보도 영역만 ROI로 잘라낸 이미지 (crosswalk_cropped)
            ROI 내부 중 특정 색 영역만 검출한 이미지 (crosswalk_threshold)
            검출된 이진 이미지의 경계만 검출한 이미지 (crosswalk_edge)
        '''
        if not self.x <= 0 and not self.y <= 0:
            cv2.line(self.cropped_image, (0, self.y), (self.crop_size_x, self.y), (0, 255, 255), 20)
        # cv2.circle(self.cropped_image, (self.x, self.y), 10, 255, -1)
        cv2.imshow("crosswalk_original", self.frame)
        cv2.imshow("crosswalk_cropped", self.cropped_image)
        cv2.imshow("crosswalk_thresholded", self.thresholded_image)
        cv2.imshow("crosswalk_edge", self.edge_image)
        # cv2.imshow("lines", self.line_detect_image)
        cv2.waitKey(1)

    # return opencv Image type
    def imageCrop(self, _img=np.ndarray(shape=(480, 640))):
        '''
            원하는 이미지 영역 검출
        '''
        self.crop_size_x = 360
        self.crop_size_y = 60
        return _img[420:480, 170:530]

    # return opencv Image type
    def edgeDetect(self, _img=np.ndarray(shape=(480, 640))):
        '''
            이미지의 경계면 검출
        '''
        return cv2.Canny(_img, 0, 360)

    def houghLineDetect(self, _img=np.ndarray(shape=(480, 640))):
        '''
            이미지의 직선 검출
        '''
        new_img = _img.copy()
        self.lines = cv2.HoughLinesP(new_img, self.RHO, self.THETA * np.pi / 180, self.THRESHOLD, minLineLength=10, maxLineGap=5)

        if self.lines is None:
            # print("length is 0")
            self.line_num = 0
        else:
            # print("length is {}".format(len(self.lines)))
            self.line_num = len(self.lines)
            for i in range(self.lines.shape[0]):
                pt1 = (self.lines[i][0][0], self.lines[i][0][1])
                pt2 = (self.lines[i][0][2], self.lines[i][0][3])    
                cv2.line(self.cropped_image, pt1, pt2, (0, 0, 255), 2, cv2.LINE_AA)

    # return opencv Image type
    def colorDetect(self, _img=np.ndarray(shape=(480, 640))):
        '''
            원하는 색 영역만 추출 (Dynamic Reconfigure를 통해, 값 변경 가능)
        '''

        hls = cv2.cvtColor(_img, cv2.COLOR_BGR2HLS)
        mask_white = cv2.inRange(hls, self.WHITE_LANE_LOW, self.WHITE_LANE_HIGH)

        return mask_white

    # ==============================================
    #               Callback Functions
    # ==============================================

    def reconfigure_callback(self, config, level):
        '''
            Dynamic_Reconfigure를 활용하여, 횡단보도 검출을 위한 색 지정
            HLS Color Space를 기반으로 검출
            흰색 횡단보도 검출을 위한 Threshold 설정
            Hough Transform 기반 직선 검출을 위한 RHO, THETA, THRESHOLD 설정
        '''
        self.WHITE_LANE_LOW = np.array([config.white_h_low, config.white_l_low, config.white_s_low])
        self.WHITE_LANE_HIGH = np.array([config.white_h_high, config.white_l_high, config.white_s_high])
        self.RHO = float(config.rho * 0.01)
        self.THETA = config.theta
        self.THRESHOLD = config.threshold
        self.CROSS_WALK_DETECT_TH = config.crosswalk_detect_threshold
        return config

    def Image_CB(self, img):
        '''
            실제 이미지를 받아서 동작하는 부분
            CompressedImage --> OpenCV Type Image 변경 (compressed_imgmsg_to_cv2)
            횡단 보도 영역만 ROI 지정 (imageCrop)
            ROI 영역에서 횡단보도 색 영역만 검출(colorDetect)
            검출된 횡단 보도의 경계면 검출 (edgeDetect)
            검출된 경계선을 이용하여 직선 검출 (houghLineDetect)
            최종 검출된 직선의 개수를 비교하여, 횡단 보도 유무 확인
            횡단 보도가 있을 경우, 흰색으로 검출된 영역의 무게 중심의 카메라 좌표계 기준 좌표 publish
        '''
        self.frame = self.cvbridge.compressed_imgmsg_to_cv2(img, "bgr8")
        self.cropped_image = self.imageCrop(self.frame)
        self.thresholded_image = self.colorDetect(self.cropped_image)
        self.edge_image = self.edgeDetect(self.thresholded_image)
        self.houghLineDetect(self.edge_image)
        self.crosswalk_distance = self.calcCrossWalkDistance(self.thresholded_image)
        self.distance_pub.publish(self.crosswalk_distance)

        # visualization
        if self.viz:
            self.visResult()


def run():
    new_class = CrossWalkDetector()
    rospy.spin()

if __name__ == '__main__':
    try:
        run()
    except KeyboardInterrupt:
        print("program down")
