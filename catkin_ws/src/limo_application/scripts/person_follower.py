#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import math
import rospy
import cv2
import numpy as np
import onnxruntime as ort
from sensor_msgs.msg import Image as RosImage
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer

# ============================ 유틸 함수들 ============================

def letterbox(im, new_shape=(640, 640), stride=32):
    """YOLOv8 스타일 레터박싱: 종횡비 유지 + 패딩"""
    h0, w0 = im.shape[:2]
    r = min(float(new_shape[0]) / h0, float(new_shape[1]) / w0)
    new_unpad = (int(round(w0 * r)), int(round(h0 * r)))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]
    dw /= 2.0
    dh /= 2.0
    im_resized = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    im_padded = cv2.copyMakeBorder(
        im_resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(114, 114, 114)
    )
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
    """간단 NMS"""
    if len(boxes) == 0:
        return []
    idxs = scores.argsort()[::-1]
    keep = []
    while len(idxs) > 0:
        i = idxs[0]
        keep.append(i)
        if len(idxs) == 1:
            break
        xx1 = np.maximum(boxes[i, 0], boxes[idxs[1:], 0])
        yy1 = np.maximum(boxes[i, 1], boxes[idxs[1:], 1])
        xx2 = np.minimum(boxes[i, 2], boxes[idxs[1:], 2])
        yy2 = np.minimum(boxes[i, 3], boxes[idxs[1:], 3])
        w = np.maximum(0, xx2 - xx1)
        h = np.maximum(0, yy2 - yy1)
        inter = w * h
        area_i = (boxes[i, 2] - boxes[i, 0]) * (boxes[i, 3] - boxes[i, 1])
        area_r = (boxes[idxs[1:], 2] - boxes[idxs[1:], 0]) * (boxes[idxs[1:], 3] - boxes[idxs[1:], 1])
        iou = inter / (area_i + area_r - inter + 1e-6)
        idxs = idxs[1:][iou <= iou_th]
    return keep

# ============================ 경량 ByteTrack-like ============================

class Track:
    _next_id = 1
    def __init__(self, box, score):
        self.id = Track._next_id
        Track._next_id += 1
        self.box = np.array(box, dtype=np.float32)  # [x1,y1,x2,y2]
        self.score = float(score)
        self.hits = 1
        self.miss = 0

def iou_xyxy(a, b):
    xx1 = max(a[0], b[0]); yy1 = max(a[1], b[1])
    xx2 = min(a[2], b[2]); yy2 = min(a[3], b[3])
    w = max(0.0, xx2 - xx1); h = max(0.0, yy2 - yy1)
    inter = w * h
    area_a = (a[2] - a[0]) * (a[3] - a[1])
    area_b = (b[2] - b[0]) * (b[3] - b[1])
    return inter / (area_a + area_b - inter + 1e-6)

class ByteTrackLite:
    """
    - IOU 기반 그리디 매칭
    - 저신뢰 생성 억제
    - miss count로 트랙 제거
    """
    def __init__(self, iou_th=0.3, max_miss=15, min_new_score=0.4):
        self.iou_th = iou_th
        self.max_miss = max_miss
        self.min_new_score = min_new_score
        self.tracks = []

    def update(self, dets_xyxy, det_scores):
        assigned = set()

        # 1) 기존 트랙에 그리디 매칭 (IOU 최대)
        for t in self.tracks:
            best_iou, best_j = 0.0, -1
            for j, (d, s) in enumerate(zip(dets_xyxy, det_scores)):
                if j in assigned:
                    continue
                iou = iou_xyxy(t.box, d)
                if iou > best_iou:
                    best_iou, best_j = iou, j
            if best_iou >= self.iou_th and best_j >= 0:
                # 업데이트 (약간의 스무딩)
                t.box = 0.7 * t.box + 0.3 * dets_xyxy[best_j]
                t.score = 0.6 * t.score + 0.4 * float(det_scores[best_j])
                t.hits += 1
                t.miss = 0
                assigned.add(best_j)
            else:
                t.miss += 1

        # 2) 매칭 안 된 검출 → 새 트랙
        for j, (d, s) in enumerate(zip(dets_xyxy, det_scores)):
            if j not in assigned and float(s) >= self.min_new_score:
                self.tracks.append(Track(d, s))

        # 3) 오래 놓친 트랙 제거
        self.tracks = [t for t in self.tracks if t.miss <= self.max_miss]

        return self.tracks

# ============================ 메인 노드 ============================

class PersonFollowerONNX:
    def __init__(self):
        rospy.init_node('person_follower_onnx', log_level=rospy.INFO)
        self.bridge = CvBridge()

        # ---- 파라미터 ----
        self.model_path = rospy.get_param('~model_path', '/home/samneotic/WeGo/catkin_ws/src/limo_application/models/yolov8m.onnx')
        self.rgb_topic  = rospy.get_param('~rgb_topic',  '/camera/rgb/image_rect_color')
        self.depth_topic= rospy.get_param('~depth_topic','/camera/depth/image_rect')
        self.desired_dist = float(rospy.get_param('~desired_dist', 0.6))
        self.max_linear  = float(rospy.get_param('~max_linear', 0.25))
        self.max_angular = float(rospy.get_param('~max_angular', 1.0))
        self.lin_k = float(rospy.get_param('~lin_k', 0.8))
        self.ang_k = float(rospy.get_param('~ang_k', 1.2))
        self.conf_th = float(rospy.get_param('~conf_th', 0.30))
        self.iou_th  = float(rospy.get_param('~iou_th', 0.50))
        self.slop = float(rospy.get_param('~slop', 0.3))
        self.queue_size = int(rospy.get_param('~queue_size', 10))
        self.roi_half = int(rospy.get_param('~roi_half', 8))  # 깊이 ROI 반경
        self.search_ang = float(rospy.get_param('~search_angular', 0.15))  # 탐색 회전 속도

        # ByteTrackLite 설정
        self.bt = ByteTrackLite(
            iou_th=float(rospy.get_param('~bt_iou', 0.3)),
            max_miss=int(rospy.get_param('~bt_max_miss', 15)),
            min_new_score=float(rospy.get_param('~bt_min_new_score', 0.4)),
        )

        # ---- ONNX 세션 ----
        self.model_path = os.path.expanduser(self.model_path)
        if not os.path.exists(self.model_path):
            rospy.logerr(f"[ONNX] model not found: {self.model_path}")
            rospy.signal_shutdown("Missing ONNX model file")
            return

        providers = ['CPUExecutionProvider']  # Nano CPU 기준. (TensorRT 사용 시 수정 필요)
        self.session = ort.InferenceSession(self.model_path, providers=providers)
        in_meta = self.session.get_inputs()[0]
        self.input_name = in_meta.name
        # 동적 입력 대비
        shp = in_meta.shape  # [N,3,H,W] or [None,3,640,640]
        if len(shp) == 4 and isinstance(shp[2], int) and isinstance(shp[3], int):
            self.input_h, self.input_w = shp[2], shp[3]
        else:
            self.input_h, self.input_w = 640, 640

        rospy.loginfo(f"[ONNX] {self.model_path}, input={self.input_name}, size=({self.input_h},{self.input_w})")

        # ---- Pub/Sub ----
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.debug_pub = rospy.Publisher('debug_image', RosImage, queue_size=1)

        rgb_sub = Subscriber(self.rgb_topic, RosImage)
        depth_sub = Subscriber(self.depth_topic, RosImage)
        ats = ApproximateTimeSynchronizer([rgb_sub, depth_sub],
                                          queue_size=self.queue_size,
                                          slop=self.slop)
        ats.registerCallback(self.callback)

        rospy.loginfo("PersonFollowerONNX (ByteTrack-lite) started.")
        rospy.spin()

    # ---------- 전처리/후처리 ----------
    def _preprocess(self, bgr):
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        img, ratio, (dw, dh) = letterbox(rgb, (self.input_h, self.input_w))
        blob = img.astype(np.float32) / 255.0
        blob = np.transpose(blob, (2, 0, 1))[None, ...]  # (1,3,h,w)
        return blob, ratio, dw, dh

    def _parse_yolov8(self, out):
        """
        YOLOv8 (비-NMS) 출력 파싱
        - out shape 예: (1, N, 84) or (1, 84, N) or (N,84)
        - 포맷: [cx, cy, w, h, obj, cls0..]
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
        # uint16: mm, float32: m
        return (float(np.median(roi)) / 1000.0) if depth.dtype == np.uint16 else float(np.median(roi))

    def _cmd(self, lin, ang):
        t = Twist()
        t.linear.x = float(np.clip(lin, -self.max_linear, self.max_linear))
        t.angular.z = float(np.clip(ang, -self.max_angular, self.max_angular))
        self.cmd_pub.publish(t)

    def _pub_debug(self, img_bgr, boxes, confs=None, cls_ids=None, sel_idx=None, dist_m=None, track_ids=None):
        vis = img_bgr.copy()
        N = 0 if boxes is None else len(boxes)
        confs = confs if confs is not None else [None] * N
        cls_ids = cls_ids if cls_ids is not None else [None] * N
        track_ids = track_ids if track_ids is not None else [None] * N

        for i in range(N):
            x1, y1, x2, y2 = map(int, boxes[i])
            color = (0, 255, 0) if (sel_idx is not None and i == sel_idx) else (0, 200, 255)
            cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
            lbl = []
            if track_ids[i] is not None:
                lbl.append(f"ID:{track_ids[i]}")
            if cls_ids[i] is not None:
                lbl.append(f"C:{cls_ids[i]}")
            if confs[i] is not None:
                lbl.append(f"{confs[i]:.2f}")
            if (sel_idx is not None and i == sel_idx) and (dist_m is not None):
                lbl.append(f"{dist_m:.2f}m")
            if lbl:
                cv2.putText(vis, " ".join(lbl), (x1, max(0, y1 - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(vis, 'bgr8'))

    # ---------- 콜백 ----------
    def callback(self, rgb_msg, depth_msg):
        # 1) ROS -> OpenCV
        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        H, W = rgb.shape[:2]

        # 2) 추론 전처리 & ONNX 추론
        blob, ratio, dw, dh = self._preprocess(rgb)
        out = self.session.run(None, {self.input_name: blob})
        boxes_xyxy, conf, cls_id = self._parse_yolov8(out)

        # 3) 사람만 남기기
        mask_person = (cls_id == 0)
        boxes_xyxy = boxes_xyxy[mask_person]
        conf = conf[mask_person]
        cls_id = cls_id[mask_person]

        # 4) 검출 없음 → 탐색 회전
        if boxes_xyxy.shape[0] == 0:
            self._cmd(0.0, self.search_ang)
            self._pub_debug(rgb, None)
            return

        # 5) 원본 좌표계로 변환 + NMS
        boxes_org = self._scale_to_original(boxes_xyxy, ratio, dw, dh, W, H)
        keep = nms_xyxy(boxes_org, conf, iou_th=self.iou_th)
        boxes_org = boxes_org[keep]; conf = conf[keep]; cls_id = cls_id[keep]

        # 6) ByteTrack-lite 업데이트
        tracks = self.bt.update(boxes_org, conf)

        if len(tracks) == 0:
            # 트랙이 아직 안정화되지 않으면 가장 큰 박스로 제어
            areas = (boxes_org[:, 2] - boxes_org[:, 0]) * (boxes_org[:, 3] - boxes_org[:, 1])
            sel = int(np.argmax(areas))
            x1, y1, x2, y2 = boxes_org[sel].astype(int)
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            dist = self._depth_median(depth, cx, cy)
            if dist is None:
                self._cmd(0.0, self.search_ang)
                self._pub_debug(rgb, boxes_org, conf, cls_id, sel_idx=sel, dist_m=None)
                return
            err_z = dist - self.desired_dist
            err_x = (cx - (W / 2.0)) / (W / 2.0)
            lin = self.lin_k * err_z
            ang = -self.ang_k * err_x
            if dist < 0.3:
                lin = min(lin, 0.0)
            self._cmd(lin, ang)
            self._pub_debug(rgb, boxes_org, conf, cls_id, sel_idx=sel, dist_m=dist)
            return

        # 7) 가장 신뢰/안정적인 트랙 선택 (hits, 면적 우선)
        tracks_sorted = sorted(
            tracks,
            key=lambda t: (t.hits, (t.box[2] - t.box[0]) * (t.box[3] - t.box[1])),
            reverse=True
        )
        t = tracks_sorted[0]
        x1, y1, x2, y2 = map(int, t.box)
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

        dist = self._depth_median(depth, cx, cy)
        if dist is None:
            self._cmd(0.0, self.search_ang)
            self._pub_debug(rgb, np.array([t.box]), np.array([t.score]), np.array([0]), sel_idx=0, dist_m=None, track_ids=[t.id])
            return

        # 8) 거리/방향 제어
        err_z = dist - self.desired_dist
        err_x = (cx - (W / 2.0)) / (W / 2.0)
        lin = self.lin_k * err_z
        ang = -self.ang_k * err_x
        if dist < 0.3:
            lin = min(lin, 0.0)

        self._cmd(lin, ang)
        self._pub_debug(rgb, np.array([t.box]), np.array([t.score]), np.array([0]), sel_idx=0, dist_m=dist, track_ids=[t.id])


if __name__ == '__main__':
    try:
        PersonFollowerONNX()
    except rospy.ROSInterruptException:
        pass
