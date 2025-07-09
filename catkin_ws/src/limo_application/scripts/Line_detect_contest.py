#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS1  - 차선 검출 노드 (HoughLinesP 버전)
  • sub : /camera/rgb/image_raw/compressed   (launch 에서 remap 가능)
  • pub : /gap     (std_msgs/Int16)   – 화면 중심과 차선 중앙의 오차
          /rotate  (std_msgs/Bool)    – 차선 미검출 시 True
"""

import math
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int16, Bool

# ---------- 색상(BGR) 및 ROI 기본치 ----------
Blue   = (255,   0,   0)
Green  = (  0, 255,   0)
Red    = (  0,   0, 255)
Yellow = (  0, 255, 255)
ROI_START_ROW = 240      #   상단 여백 (픽셀)
ROI_END_ROW   = 480      #   하단 여백 (픽셀)
L_ROW         = 100      #   ROI 내부 기준선 y 좌표 (ROI 기준)
LANE_GAP_PX   = 380      #   좌·우 차선 간 기대 너비(px)

class LineDetectHL:
    def __init__(self):
        # ---------- ROS I/O ----------
        self.img_sub_  = rospy.Subscriber(
            'camera/rgb/image_raw/compressed',
            CompressedImage, self.img_cb, queue_size=1, buff_size=2**24)

        self.error_pub_  = rospy.Publisher('gap',    Int16, queue_size=10)
        self.rotate_pub_ = rospy.Publisher('rotate', Bool,  queue_size=10)

        # ---------- 내부 상태 ----------
        self.br   = CvBridge()
        self.prev_x_left  = 0
        self.prev_x_right = 0
        self.debug = rospy.get_param('~debug', False)

    # ───────────────────────────────────────────────────────── img_cb
    def img_cb(self, msg):
        # 압축 -> cv2 이미지
        frame = self.br.compressed_imgmsg_to_cv2(msg, 'bgr8')
        height, width = frame.shape[:2]

        success, x_left, x_right, dbg = self.detect_lanes(frame)

        # ---------- rotate / error 토픽 ----------
        self.rotate_pub_.publish(Bool(data = not success))

        err = Int16()
        if success:
            midpoint     = (x_left + x_right) // 2
            view_center  = width // 2
            err.data     = view_center - midpoint
        else:
            err.data     = 0
        self.error_pub_.publish(err)

        # ---------- 디버깅 화면 ----------
        if self.debug and dbg is not None:
            cv2.imshow('lane_debug', dbg)
            cv2.waitKey(1)

    # ───────────────────────────────────────────────────── detect_lanes
    def detect_lanes(self, image):
        """
        image : BGR 원본 전체 프레임
        return (success, x_left, x_right, dbg_img)
        """
        height, width = image.shape[:2]
        roi = image[ROI_START_ROW:ROI_END_ROW, 0:width]
        roi_h = ROI_END_ROW - ROI_START_ROW
        view_center = width // 2

        # ----- 전처리 : Canny 에지 -----
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        edges = cv2.Canny(blur, 60, 75)

        lines = cv2.HoughLinesP(edges, 1, math.pi/180,
                                threshold=50, minLineLength=50, maxLineGap=20)
        if lines is None :
            return False, 0, 0, None

        # ----- 기울기·위치로 좌/우 선분 분류 -----
        left_lines, right_lines = [], []
        for ln in lines:
            x1,y1,x2,y2 = ln[0]
            slope = float('inf') if x2==x1 else (y2-y1)/(x2-x1)
            if abs(slope) < 0.2:      # 수평선 제거
                continue
            if  slope < 0 and x2 <  view_center:  left_lines.append([x1,y1,x2,y2])
            if  slope > 0 and x1 >  view_center:  right_lines.append([x1,y1,x2,y2])

        # 대표직선(m,b) 계산 함수
        def rep_line(segs):
            if not segs:
                return 0.0, 0.0
            x_sum=y_sum=m_sum=0.0
            for x1,y1,x2,y2 in segs:
                x_sum += x1+x2
                y_sum += y1+y2
                m_sum += 0 if x2==x1 else (y2-y1)/(x2-x1)
            n=len(segs)
            x_avg = x_sum/(2*n); y_avg = y_sum/(2*n)
            m = m_sum/n;  b = y_avg - m*x_avg
            return m,b

        m_left,  b_left  = rep_line(left_lines)
        m_right, b_right = rep_line(right_lines)

        # 교점(x 좌표) 구하기.  없으면 이전값 사용
        x_left  = self.prev_x_left  if m_left  == 0 else int((L_ROW-b_left) / m_left)
        x_right = self.prev_x_right if m_right == 0 else int((L_ROW-b_right)/ m_right)

        # 한쪽만 검출된 경우 예상값으로 보정
        if m_left==0 and m_right!=0:   x_left  = x_right - LANE_GAP_PX
        if m_left!=0 and m_right==0:   x_right = x_left  + LANE_GAP_PX

        # 이전값 갱신
        self.prev_x_left, self.prev_x_right = x_left, x_right

        # ---------- 디버그 캔버스 ----------
        dbg = None
        if self.debug:
            dbg = image.copy()
            cv2.rectangle(dbg, (0,ROI_START_ROW),
                               (width,ROI_END_ROW), (60,60,60), 1)
            # ROI 내부 그리기
            roi_dbg = roi.copy()
            # 원본 선분
            for x1,y1,x2,y2 in left_lines:  cv2.line(roi_dbg,(x1,y1),(x2,y2),Red,2)
            for x1,y1,x2,y2 in right_lines: cv2.line(roi_dbg,(x1,y1),(x2,y2),Yellow,2)
            # 대표직선
            if m_left!=0:
                cv2.line(roi_dbg,(x_left,0),(x_left,roi_h),Blue,2)
            if m_right!=0:
                cv2.line(roi_dbg,(x_right,0),(x_right,roi_h),Blue,2)
            # 기준·중앙점
            midpoint=(x_left+x_right)//2
            cv2.line(roi_dbg,(0,L_ROW),(width,L_ROW),Yellow,2)
            for x,c in [(x_left,Green),(x_right,Green),
                        (midpoint,Blue),(view_center,Red)]:
                cv2.rectangle(roi_dbg,(x-4,L_ROW-4),(x+4,L_ROW+4),c,2)
            dbg[ROI_START_ROW:ROI_END_ROW,0:width]=roi_dbg

        return True, x_left, x_right, dbg

# ─────────────────────────────────────────────────────────────── main
def main():
    rospy.init_node('line_detect_hough')
    LineDetectHL()
    rospy.spin()

if __name__ == '__main__':
    main()
