#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

class YoloV3NanoNode:
    def __init__(self):
        # ----- ROS Params -----
        self.cfg_path      = rospy.get_param('~cfg_path',  '$(find limo_yolo)/yolo/yolov3n.cfg')
        self.weights_path  = rospy.get_param('~weights_path','$(find limo_yolo)/yolo/yolov3n.weights')
        self.names_path    = rospy.get_param('~names_path', '$(find limo_yolo)/yolo/coco.names')
        self.conf_thresh   = rospy.get_param('~conf_thresh', 0.4)
        self.nms_thresh    = rospy.get_param('~nms_thresh', 0.45)
        self.target_w      = rospy.get_param('~input_width', 416)
        self.target_h      = rospy.get_param('~input_height', 416)
        self.print_limit   = rospy.get_param('~print_limit', 5)  # 한 프레임 최대 몇 개까지 출력
        self.draw          = rospy.get_param('~draw', True)
        self.publish_image = rospy.get_param('~publish_image', True)
        self.class_filter  = rospy.get_param('~class_filter', [])  # [] = 전부 허용, ex ['person','car']
        self.frame_skip    = rospy.get_param('~frame_skip', 0)     # 처리 부하 줄이려면 1=>1프레임 건너뜀 등

        resolve = rospy.resolve_name  # $(find ...) 확장 (roslaunch substitution) 위해
        # (roslaunch substitution 은 rospy.get_param 에선 자동 확장 안됨 -> 아래 간단 처리)
        for attr in ['cfg_path','weights_path','names_path']:
            val = getattr(self, attr)
            if val.startswith('$(find '):
                pkg = val.split('$(find ')[1].split(')')[0]
                import rospkg, os
                p = rospkg.RosPack().get_path(pkg)
                rest = val.split(')')[1]
                setattr(self, attr, p + rest)

        # ----- Load class names -----
        with open(self.names_path, 'r') as f:
            self.class_names = [c.strip() for c in f.readlines() if c.strip()]

        # ----- Load network -----
        rospy.loginfo("Loading YOLOv3n cfg=%s weights=%s", self.cfg_path, self.weights_path)
        self.net = cv2.dnn.readNetFromDarknet(self.cfg_path, self.weights_path)
        # CPU
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        self.out_layer_names = self.net.getUnconnectedOutLayersNames()

        # Sub / Pub
        self.sub = rospy.Subscriber('/limo/color/image_raw/compressed',
                                    CompressedImage, self.image_cb,
                                    queue_size=1, buff_size=2**22)
        if self.publish_image:
            self.pub_annot = rospy.Publisher('~annotated', Image, queue_size=1)
            try:
                from cv_bridge import CvBridge
                self.bridge = CvBridge()
            except ImportError:
                rospy.logwarn("cv_bridge 미설치: publish_image=false 로 설정하거나 설치하세요.")
                self.publish_image = False

        self.pub_text = rospy.Publisher('~detections_text', String, queue_size=5)

        self.frame_count = 0
        rospy.loginfo("[yolo_v3n_node] Ready.")

    def image_cb(self, msg: CompressedImage):
        self.frame_count += 1
        if self.frame_skip > 0 and (self.frame_count % (self.frame_skip+1) != 1):
            return

        # --- Decode compressed image ---
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if img is None:
            rospy.logwarn("이미지 디코드 실패")
            return

        blob = cv2.dnn.blobFromImage(img, 1/255.0,
                                     (self.target_w, self.target_h),
                                     swapRB=True, crop=False)
        self.net.setInput(blob)
        layer_outs = self.net.forward(self.out_layer_names)

        h, w = img.shape[:2]
        boxes = []
        confidences = []
        class_ids = []

        # --- Parse YOLO outputs ---
        for out in layer_outs:
            for det in out:
                scores = det[5:]
                class_id = np.argmax(scores)
                conf = scores[class_id] * det[4]  # det[4]=objectness
                if conf < self.conf_thresh:
                    continue
                cls_name = self.class_names[class_id] if class_id < len(self.class_names) else str(class_id)
                if self.class_filter and cls_name not in self.class_filter:
                    continue
                # bbox center x,y,w,h (YOLO normalized)
                cx, cy, bw, bh = det[0:4]
                x = int((cx - bw/2) * w)
                y = int((cy - bh/2) * h)
                bw_px = int(bw * w)
                bh_px = int(bh * h)
                boxes.append([x, y, bw_px, bh_px])
                confidences.append(float(conf))
                class_ids.append(class_id)

        # --- NMS ---
        idxs = cv2.dnn.NMSBoxes(boxes, confidences, self.conf_thresh, self.nms_thresh)

        printed = 0
        lines = []
        if len(idxs) > 0:
            for i in idxs.flatten():
                if printed >= self.print_limit:
                    break
                cls = self.class_names[class_ids[i]] if class_ids[i] < len(self.class_names) else str(class_ids[i])
                c  = confidences[i]
                x,y,bw_px,bh_px = boxes[i]
                lines.append(f"{cls} {c:.2f} ({x},{y},{bw_px},{bh_px})")
                printed += 1
                if self.draw:
                    cv2.rectangle(img, (x,y), (x+bw_px,y+bh_px), (0,255,0), 2)
                    cv2.putText(img, f"{cls}:{c:.2f}", (x, y-5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

        if not lines:
            lines.append("NO DETECTION")

        # Print to terminal
        rospy.loginfo_throttle(0.5, "[YOLO] " + " | ".join(lines))

        # Publish text
        self.pub_text.publish(String(" | ".join(lines)))

        # Publish annotated image (optional)
        if self.publish_image and self.draw:
            img_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            img_msg.header.stamp = msg.header.stamp
            img_msg.header.frame_id = "camera"
            self.pub_annot.publish(img_msg)

def main():
    rospy.init_node('yolo_v3n_node')
    YoloV3NanoNode()
    rospy.spin()

if __name__ == '__main__':
    main()
