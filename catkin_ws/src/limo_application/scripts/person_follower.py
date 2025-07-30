#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, cv2, numpy as np, onnxruntime as ort
from sensor_msgs.msg import Image as RosImage
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
import math

# -------------------- 유틸 --------------------
def letterbox(im, new_shape=(640, 640), stride=32):
    h0, w0 = im.shape[:2]
    r = min(float(new_shape[0]) / h0, float(new_shape[1]) / w0)
    new_unpad = (int(round(w0 * r)), int(round(h0 * r)))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]
    dw /= 2.0; dh /= 2.0
    im_resized = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    im_padded = cv2.copyMakeBorder(im_resized, top, bottom, left, right,
                                   cv2.BORDER_CONSTANT, value=(114,114,114))
    return im_padded, r, (left, top)

def xywh2xyxy(xywh):
    xyxy = xywh.copy()
    xyxy[:, 0] = xywh[:, 0] - xywh[:, 2] / 2.0
    xyxy[:, 1] = xywh[:, 1] - xywh[:, 3] / 2.0
    xyxy[:, 2] = xywh[:, 0] + xywh[:, 2] / 2.0
    xyxy[:, 3] = xywh[:, 1] + xywh[:, 3] / 2.0
    return xyxy

def clip_boxes(xyxy, w, h):
    xyxy[:, 0] = np.clip(xyxy[:, 0], 0, w - 1)
    xyxy[:, 1] = np.clip(xyxy[:, 1], 0, h - 1)
    xyxy[:, 2] = np.clip(xyxy[:, 2], 0, w - 1)
    xyxy[:, 3] = np.clip(xyxy[:, 3], 0, h - 1)
    return xyxy

def nms_xyxy(boxes, scores, iou_th=0.5):
    if len(boxes) == 0:
        return []
    idxs = scores.argsort()[::-1]
    keep = []
    while len(idxs) > 0:
        i = idxs[0]
        keep.append(i)
        if len(idxs) == 1:
            break
        xx1 = np.maximum(boxes[i,0], boxes[idxs[1:],0])
        yy1 = np.maximum(boxes[i,1], boxes[idxs[1:],1])
        xx2 = np.minimum(boxes[i,2], boxes[idxs[1:],2])
        yy2 = np.minimum(boxes[i,3], boxes[idxs[1:],3])
        w = np.maximum(0, xx2 - xx1)
        h = np.maximum(0, yy2 - yy1)
        inter = w * h
        area_i = (boxes[i,2]-boxes[i,0])*(boxes[i,3]-boxes[i,1])
        area_r = (boxes[idxs[1:],2]-boxes[idxs[1:],0])*(boxes[idxs[1:],3]-boxes[idxs[1:],1])
        iou = inter / (area_i + area_r - inter + 1e-6)
        idxs = idxs[1:][iou <= iou_th]
    return keep

# -------------------- 메인 노드 --------------------
class PersonFollowerONNX:
    def __init__(self):
        rospy.init_node('person_follower_onnx', log_level=rospy.INFO)
        self.bridge = CvBridge()

        # 파라미터/home/samneotic/WeGo/catkin_ws/src/limo_application/models
        self.model_path = rospy.get_param('~model_path', '/home/samneotic/WeGo/catkin_ws/src/limo_application/models/yolov8m.onnx')
        self.rgb_topic = rospy.get_param('~rgb_topic', '/camera/rgb/image_rect_color')
        self.depth_topic = rospy.get_param('~depth_topic', '/camera/depth/image_rect')
        self.desired_dist = float(rospy.get_param('~desired_dist', 0.6))
        self.max_linear = float(rospy.get_param('~max_linear', 0.30))
        self.max_angular = float(rospy.get_param('~max_angular', 1.0))
        self.lin_k = float(rospy.get_param('~lin_k', 0.8))
        self.ang_k = float(rospy.get_param('~ang_k', 1.2))
        self.conf_th = float(rospy.get_param('~conf_th', 0.30))
        self.iou_th = float(rospy.get_param('~iou_th', 0.50))
        self.slop = float(rospy.get_param('~slop', 0.3))
        self.queue_size = int(rospy.get_param('~queue_size', 10))
        self.roi_half = int(rospy.get_param('~roi_half', 6))  # 깊이 ROI 반경(픽셀)

        # ONNX 세션
        providers = ['CPUExecutionProvider']  # Nano CPU라면 이대로, TensorRT면 별도
        self.session = ort.InferenceSession(self.model_path, providers=providers)
        in_meta = self.session.get_inputs()[0]
        self.input_name = in_meta.name

        # 동적 입력(shape에 -1 있을 수 있음) 처리
        shp = in_meta.shape  # [N,3,H,W] 혹은 [None,3,640,640]
        if len(shp) == 4 and isinstance(shp[2], int) and isinstance(shp[3], int):
            self.input_h, self.input_w = shp[2], shp[3]
        else:
            self.input_h, self.input_w = 640, 640  # 기본값
        rospy.loginfo(f"[ONNX] {self.model_path}, input={self.input_name}, size=({self.input_h},{self.input_w})")

        # 퍼블리셔
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.debug_pub = rospy.Publisher('debug_image', RosImage, queue_size=1)

        # 구독(시간동기)
        rgb_sub = Subscriber(self.rgb_topic, RosImage)
        depth_sub = Subscriber(self.depth_topic, RosImage)
        ats = ApproximateTimeSynchronizer([rgb_sub, depth_sub],
                                          queue_size=self.queue_size,
                                          slop=self.slop)
        ats.registerCallback(self.callback)

        # 트래커
        self.tracker = None
        self.following = False
        self._tracker_ctor = self._get_tracker_ctor()

        rospy.loginfo("PersonFollowerONNX started.")
        rospy.spin()

    def _get_tracker_ctor(self):
        if hasattr(cv2, 'TrackerCSRT_create'):
            return cv2.TrackerCSRT_create
        if hasattr(cv2, 'legacy') and hasattr(cv2.legacy, 'TrackerCSRT_create'):
            return cv2.legacy.TrackerCSRT_create
        rospy.logwarn("CSRT tracker not found. Falling back to KCF.")
        if hasattr(cv2, 'TrackerKCF_create'):
            return cv2.TrackerKCF_create
        if hasattr(cv2, 'legacy') and hasattr(cv2.legacy, 'TrackerKCF_create'):
            return cv2.legacy.TrackerKCF_create
        rospy.logerr("No OpenCV tracker available.")
        return None

    def _preprocess(self, bgr):
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        img, ratio, (dw, dh) = letterbox(rgb, (self.input_h, self.input_w))
        blob = img.astype(np.float32) / 255.0
        blob = np.transpose(blob, (2, 0, 1))[None, ...]
        return blob, ratio, dw, dh

    def _parse_yolov8(self, out):
        """
        out: typical shapes
          - (1, N, 84) or (N, 84)
          - (1, 84, N) -> transpose to (N, 84)
        format: [cx, cy, w, h, obj, cls0..]
        """
        if isinstance(out, (list, tuple)):
            out = out[0]
        if out.ndim == 3:
            out = out[0]
        if out.shape[0] == 84 and out.shape[1] != 84:
            out = out.T  # (N,84)
        if out.shape[1] < 6:
            rospy.logwarn(f"Unexpected YOLO output shape: {out.shape}")
            return np.empty((0,4), np.float32), np.empty((0,), np.float32), np.empty((0,), np.int32)

        boxes_xywh = out[:, :4]
        obj = out[:, 4]
        cls_scores = out[:, 5:]
        cls_id = np.argmax(cls_scores, axis=1)
        cls_prob = cls_scores[np.arange(cls_scores.shape[0]), cls_id]
        conf = obj * cls_prob

        keep = conf > self.conf_th
        if not np.any(keep):
            return np.empty((0,4), np.float32), np.empty((0,), np.float32), np.empty((0,), np.int32)

        boxes_xywh = boxes_xywh[keep]
        conf = conf[keep]
        cls_id = cls_id[keep]

        boxes_xyxy = xywh2xyxy(boxes_xywh)
        return boxes_xyxy.astype(np.float32), conf.astype(np.float32), cls_id.astype(np.int32)

    def _scale_to_original(self, boxes_xyxy, ratio, dw, dh, orig_w, orig_h):
        boxes = boxes_xyxy.copy()
        boxes[:, [0, 2]] -= dw
        boxes[:, [1, 3]] -= dh
        boxes[:, :4] /= ratio
        boxes = clip_boxes(boxes, orig_w, orig_h)
        return boxes

    def _pub_debug(self, img_bgr, boxes, confs, cls_ids, sel_idx=None, dist_m=None):
        vis = img_bgr.copy()
        for i, b in enumerate(boxes):
            x1, y1, x2, y2 = map(int, b)
            color = (0, 255, 0) if i == sel_idx else (0, 200, 255)
            cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
            label = f"id:{cls_ids[i]} conf:{confs[i]:.2f}"
            if i == sel_idx and dist_m is not None:
                label += f" {dist_m:.2f}m"
            cv2.putText(vis, label, (x1, max(0, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(vis, 'bgr8'))

    def _cmd(self, lin, ang):
        t = Twist()
        t.linear.x = float(np.clip(lin, -self.max_linear, self.max_linear))
        t.angular.z = float(np.clip(ang, -self.max_angular, self.max_angular))
        self.cmd_pub.publish(t)

    def _depth_median(self, depth, cx, cy):
        h, w = depth.shape[:2]
        r = self.roi_half
        x1 = max(cx - r, 0); x2 = min(cx + r, w - 1)
        y1 = max(cy - r, 0); y2 = min(cy + r, h - 1)
        roi = depth[y1:y2+1, x1:x2+1].flatten()
        roi = roi[np.isfinite(roi)]
        roi = roi[roi > 0]
        if roi.size == 0:
            return None
        # 단위: uint16이면 mm, float32이면 m
        return (float(np.median(roi)) / 1000.0) if depth.dtype == np.uint16 else float(np.median(roi))

    def callback(self, rgb_msg, depth_msg):
        # ROS -> OpenCV
        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        H, W = rgb.shape[:2]

        # 추적 모드
        if self.following and self.tracker is not None:
            ok, bbox = self.tracker.update(rgb)
            if not ok:
                self.following = False
            else:
                x, y, w, h = [int(v) for v in bbox]
                cx, cy = x + w // 2, y + h // 2
                dist = self._depth_median(depth, cx, cy)

                if dist is None:
                    # 깊이 없음: 천천히 스캔
                    self._cmd(0.0, 0.2)
                    self._pub_debug(rgb, np.array([[x, y, x + w, y + h]]), np.array([1.0]), np.array([0]), 0, None)
                    return

                # 제어
                err_z = dist - self.desired_dist
                err_x = (cx - (W / 2.0)) / (W / 2.0)
                lin = self.lin_k * err_z
                ang = -self.ang_k * err_x

                if dist < 0.3:
                    lin = min(lin, 0.0)  # 너무 가까우면 더 다가가지 않음

                self._cmd(lin, ang)
                self._pub_debug(rgb, np.array([[x, y, x + w, y + h]]), np.array([1.0]), np.array([0]), 0, dist)
                return

        # 검출 모드
        blob, ratio, dw, dh = self._preprocess(rgb)
        out = self.session.run(None, {self.input_name: blob})
        boxes_xyxy, conf, cls_id = self._parse_yolov8(out)

        # 사람만
        mask_person = (cls_id == 0)
        boxes_xyxy = boxes_xyxy[mask_person]
        conf = conf[mask_person]
        cls_id = cls_id[mask_person]

        if boxes_xyxy.shape[0] == 0:
            # 검출 없음: 서서 좌우 탐색
            self._cmd(0.0, 0.15)
            self._pub_debug(rgb, np.empty((0, 4)), np.empty((0,)), np.empty((0,)))
            return

        # 원본 좌표로 역투영
        boxes_org = self._scale_to_original(boxes_xyxy, ratio, dw, dh, W, H)

        # NMS 후 최종 1개
        keep = nms_xyxy(boxes_org, conf, iou_th=self.iou_th)
        boxes_org = boxes_org[keep]; conf = conf[keep]; cls_id = cls_id[keep]

        areas = (boxes_org[:, 2] - boxes_org[:, 0]) * (boxes_org[:, 3] - boxes_org[:, 1])
        sel = int(np.argmax(areas))
        x1, y1, x2, y2 = boxes_org[sel].astype(int)
        w = x2 - x1; h = y2 - y1

        # 트래커 초기화
        if self._tracker_ctor is None:
            rospy.logerr("No available OpenCV tracker.")
            return
        self.tracker = self._tracker_ctor()
        ok = self.tracker.init(rgb, (int(x1), int(y1), int(w), int(h)))
        self.following = bool(ok)
        self._pub_debug(rgb, boxes_org, conf, cls_id, sel_idx=sel, dist_m=None)
        rospy.loginfo(f"Tracker init: {ok}, bbox={(x1, y1, w, h)}")


if __name__ == '__main__':
    try:
        PersonFollowerONNX()
    except rospy.ROSInterruptException:
        pass
