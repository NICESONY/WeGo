#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
카메라(CompressedImage) → gap(Int16) · rotate(Bool)
"""

import rospy, cv2, cv_bridge, numpy as np, yaml
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg    import Int16, Bool

class TwoLineDetect:
    def __init__(self):
        #  ROS I/O
        self.img_sub_ = rospy.Subscriber(
            "camera/rgb/image_raw/compressed",
            CompressedImage,
            self.img_cb,
            queue_size=1,
            buff_size=2**24)

        self.error_pub_  = rospy.Publisher("gap",    Int16, queue_size=10)
        self.rotate_pub_ = rospy.Publisher("rotate", Bool,   queue_size=10)
        self.debug_pub_ = rospy.Publisher("debug_lane", Image, queue_size=1)   # ★ 추가
        self.br_ = cv_bridge.CvBridge()

        #  파라미터 
        def _to_list(param, default):
            v = rospy.get_param(param, default)
            if isinstance(v, str):
                v = yaml.safe_load(v)          # 문자열 → 리스트 변환
            return list(map(int, v))

        self.lower_yel = np.array(_to_list("~lower_yellow", [15, 80, 80]),  np.uint8)
        self.upper_yel = np.array(_to_list("~upper_yellow", [35,255,255]), np.uint8)
        self.roi_ratio = rospy.get_param("~roi_ratio", 0.7)    # 화면 아래 40 %

    #  콜백 
    def img_cb(self, msg: CompressedImage):
        frame = self.br_.compressed_imgmsg_to_cv2(msg, "bgr8")
        h, w  = frame.shape[:2]
        roi   = frame[int(h*self.roi_ratio):, :]

        hsv  = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_yel, self.upper_yel)
        mask = cv2.GaussianBlur(mask, (5,5), 0)

        edges = cv2.Canny(mask, 50, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180,
                                threshold=50, minLineLength=50, maxLineGap=50)

        # ─ 차선 미검출 ─
        if lines is None:
            self.rotate_pub_.publish(True)
            self.error_pub_.publish(0)
            return

        left, right = [], []
        for x1,y1,x2,y2 in lines[:,0]:
            slope = (y2-y1) / (x2-x1+1e-6)
            if abs(slope) < 0.3:
                continue
            if slope < 0 and x1 < w/2 and x2 < w/2:
                left.append((x1,y1,x2,y2))
            elif slope > 0 and x1 > w/2 and x2 > w/2:
                right.append((x1,y1,x2,y2))

        if not left or not right:
            self.rotate_pub_.publish(True)
            self.error_pub_.publish(0)
            return

        # ─ 중앙 계산 ─
        lx, ly = self._avg_line(left)
        rx, ry = self._avg_line(right)
        l_fit  = np.polyfit(lx, ly, 1)
        r_fit  = np.polyfit(rx, ry, 1)

        # ★ 직접 그릴 y=0 지점의 x좌표 추가 계산
        lx_top = -l_fit[1] / l_fit[0]        # y = 0 → x
        rx_top = -r_fit[1] / r_fit[0]


        y_eval = roi.shape[0]-1
        lx_btm = (y_eval - l_fit[1]) / l_fit[0]
        rx_btm = (y_eval - r_fit[1]) / r_fit[0]
        x_mid  = (lx_btm + rx_btm) / 2.0

        err_px = int(round(x_mid - w/2))

        self.rotate_pub_.publish(False)
        self.error_pub_.publish(err_px)

        vis = roi.copy()
        # 왼쪽·오른쪽 최종 직선 그리기
        cv2.line(vis, (int(lx_btm), vis.shape[0]-1), (int(lx_top), 0), (0,255,0), 2)
        cv2.line(vis, (int(rx_btm), vis.shape[0]-1), (int(rx_top), 0), (0,255,0), 2)
        # 중앙점
        cv2.circle(vis, (int(x_mid), int(vis.shape[0]-1)), 4, (0,0,255), -1)

        # 퍼블리시
        self.debug_pub_.publish(self.br_.cv2_to_imgmsg(vis, "bgr8"))

    # 헬퍼 
    @staticmethod
    def _avg_line(pts):
        xs = [x1 for x1,_,x2,_ in pts] + [x2 for x1,_,x2,_ in pts]
        ys = [y1 for _,y1,_,y2 in pts] + [y2 for _,y1,_,y2 in pts]
        return np.array(xs), np.array(ys)


if __name__ == "__main__":
    rospy.init_node("Two_Line_detect")
    TwoLineDetect()
    rospy.loginfo("Two_Line_detect node started")
    rospy.spin()
