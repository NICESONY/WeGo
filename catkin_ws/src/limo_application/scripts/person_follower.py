#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np

class YoloFollower:
    def __init__(self):
        rospy.init_node('yolo8n_follower')
        self.bridge = CvBridge()

        # 모델 로드 (로컬에 yolov8n.pt 파일이 있어야 합니다)
        model_path = rospy.get_param('~model_path', 'yolov8n.pt')
        self.model = YOLO(model_path)

        # 퍼블리셔/서브스크라이버
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw',
                                          Image, self.cb_image, queue_size=1)

        # 제어 파라미터
        self.K_ang     = rospy.get_param('~K_ang',  1.0)
        self.K_lin     = rospy.get_param('~K_lin',  0.5)
        self.target_hf = rospy.get_param('~target_hf', 0.4)  # 목표 bbox 높이 비율
        self.max_lin   = rospy.get_param('~max_lin', 0.2)
        self.max_ang   = rospy.get_param('~max_ang', 1.0)

        rospy.loginfo("[YoloFollower] started")
        rospy.spin()

    def cb_image(self, msg: Image):
        # 1) ROS Image → CV2 BGR
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        H, W = img.shape[:2]

        # 2) YOLO 추론
        results = self.model(img, imgsz=(640,640))[0]  # first frame
        boxes = results.boxes.xyxy.cpu().numpy()       # (N,4)
        cls  = results.boxes.cls.cpu().numpy().astype(int)
        conf = results.boxes.conf.cpu().numpy()

        # 3) 사람(class 0) 필터
        mask = (cls == 0)
        boxes = boxes[mask]
        conf  = conf[mask]
        if boxes.shape[0] == 0:
            # 감지 없으면 제자리 회전
            self.publish_cmd(0.0, 0.5)
            return

        # 4) 가장 높은 confidence 선택
        idx = np.argmax(conf)
        x1, y1, x2, y2 = boxes[idx]
        # 박스 중심 편향
        cx = (x1 + x2) / 2.0
        err_x = (cx - W/2.0) / (W/2.0)  # -1 ~ +1

        # 5) bbox 높이 비율로 거리 제어
        box_h = (y2 - y1)
        hf = box_h / H
        err_h = self.target_hf - hf

        # 6) 제어량 계산
        ang = float(np.clip(self.K_ang * (-err_x), -self.max_ang, self.max_ang))
        lin = float(np.clip(self.K_lin * err_h, -self.max_lin, self.max_lin))

        self.publish_cmd(lin, ang)

    def publish_cmd(self, lin, ang):
        t = Twist()
        t.linear.x  = lin
        t.angular.z = ang
        self.cmd_pub.publish(t)

if __name__ == '__main__':
    try:
        YoloFollower()
    except rospy.ROSInterruptException:
        pass
