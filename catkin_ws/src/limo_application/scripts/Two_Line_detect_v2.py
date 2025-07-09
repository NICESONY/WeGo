#!/usr/bin/env python3
# Two_Line_detect_v2.py  (ROS1 Noetic, Python3)
import rospy, cv2, cv_bridge, numpy as np, yaml
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg    import Int16, Bool, Float32

# ───────────── 파라미터 기본값 ─────────────
TH_SOBELX  = (35, 100)           # :contentReference[oaicite:1]{index=1}
TH_SOBELY  = (30, 255)
TH_MAG     = (30, 255)
TH_DIR     = (0.7, 1.3)
TH_H, TH_L, TH_S = (160,255), (50,160), (0,255)

SRC_PTS = np.float32([[270, 40],[310, 40],[450, 720],[100, 720]])   # :contentReference[oaicite:2]{index=2}
DST_PTS = np.float32([[250, 0],[510, 0],[510, 720],[250, 720]])

class LaneDetector:
    def __init__(self):
        # ─ ROS IO ─
        self.sub = rospy.Subscriber("camera/rgb/image_raw/compressed",
                                    CompressedImage, self.cb,  queue_size=1, buff_size=2**24)
        self.pub_gap  = rospy.Publisher("gap",    Int16,  queue_size=10)
        self.pub_rot  = rospy.Publisher("rotate", Bool,   queue_size=10)
        self.pub_dbg  = rospy.Publisher("debug_lane", Image, queue_size=1)
        self.pub_curv = rospy.Publisher("curvature", Float32, queue_size=10)  # 추가 옵션

        self.br = cv_bridge.CvBridge()
        self.M   = cv2.getPerspectiveTransform(SRC_PTS, DST_PTS)
        self.Minv= cv2.getPerspectiveTransform(DST_PTS, SRC_PTS)

    # ───────── 콜백 ─────────
    def cb(self, msg):
        frame = self.br.compressed_imgmsg_to_cv2(msg, "bgr8")
        bin_comb = self.binary_pipeline(frame)
        warp = cv2.warpPerspective(bin_comb, self.M, (720,720))

        ret, left_fit, right_fit, dbg_warp = self.sliding_window(warp)

        # 미검출
        if not ret:
            self.pub_rot.publish(True)
            self.pub_gap.publish(0)
            self.pub_dbg.publish(self.br.cv2_to_imgmsg(frame, "bgr8"))
            return

        # 중앙 오차 (px)
        h, w = frame.shape[:2]
        y_eval = h
        lx = np.polyval(left_fit, y_eval)
        rx = np.polyval(right_fit, y_eval)
        lane_mid = (lx + rx) / 2
        gap_px = int(round(lane_mid - w/2))

        # 곡률 (meter-단위 변환 생략, px 단위 출력)
        left_curv = ((1 + (2*left_fit[0]*y_eval + left_fit[1])**2)**1.5) / abs(2*left_fit[0])
        right_curv= ((1 + (2*right_fit[0]*y_eval+ right_fit[1])**2)**1.5) / abs(2*right_fit[0])
        curv = float((left_curv + right_curv)/2)

        # 퍼블리시
        self.pub_rot.publish(False)
        self.pub_gap.publish(gap_px)
        self.pub_curv.publish(curv)

        # 원근 복원 & 시각화
        lane_overlay = self.draw_lane(frame, warp, left_fit, right_fit)
        self.pub_dbg.publish(self.br.cv2_to_imgmsg(lane_overlay, "bgr8"))

    # ───────── 1. 이진화 파이프라인 ─────────
    def binary_pipeline(self, img):
        hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        H,L,S = hls[:,:,0], hls[:,:,1], hls[:,:,2]

        # Sobel x,y
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        sobelx = np.absolute(cv2.Sobel(gray, cv2.CV_64F, 1,0))
        sobely = np.absolute(cv2.Sobel(gray, cv2.CV_64F, 0,1))
        scale = np.max(sobelx); sobelx = (sobelx/scale*255).astype(np.uint8)
        scale = np.max(sobely); sobely = (sobely/scale*255).astype(np.uint8)

        sobelx_bin = ((sobelx>=TH_SOBELX[0])&(sobelx<=TH_SOBELX[1]))
        sobely_bin = ((sobely>=TH_SOBELY[0])&(sobely<=TH_SOBELY[1]))

        # magnitude / direction
        gradmag = np.sqrt(sobelx**2 + sobely**2); gradmag = (gradmag/np.max(gradmag)*255).astype(np.uint8)
        mag_bin = ((gradmag>=TH_MAG[0])&(gradmag<=TH_MAG[1]))

        graddir = np.arctan2(np.absolute(sobely), np.absolute(sobelx))
        dir_bin = ((graddir>=TH_DIR[0])&(graddir<=TH_DIR[1]))

        grad_comb = ((sobelx_bin & sobely_bin) | (sobelx_bin & mag_bin & dir_bin))

        # HLS 조건  :contentReference[oaicite:3]{index=3}
        h_bin = ((H>TH_H[0])&(H<=TH_H[1]))
        l_bin = ((L>TH_L[0])&(L<=TH_L[1]))
        s_bin = ((S>TH_S[0])&(S<=TH_S[1]))
        hls_comb = (((s_bin)&(~l_bin)) | ((~s_bin)&h_bin&l_bin))

        combined = np.zeros_like(gray, dtype=np.uint8)
        combined[(grad_comb|hls_comb)] = 255
        return combined

    # ───────── 2. 슬라이딩 윈도우 & 피팅 ─────────
    def sliding_window(self, binary_warped, nwindows=9, margin=56, minpix=50):
        histogram = np.sum(binary_warped[binary_warped.shape[0]//2:,:], axis=0)
        midpoint = histogram.shape[0]//2
        leftx_base = np.argmax(histogram[:midpoint]); rightx_base = np.argmax(histogram[midpoint:])+midpoint

        window_h = binary_warped.shape[0]//nwindows
        nonzero = binary_warped.nonzero()
        nz_y, nz_x = np.array(nonzero[0]), np.array(nonzero[1])
        leftx_cur, rightx_cur = leftx_base, rightx_base
        left_inds, right_inds = [], []

        for win in range(nwindows):
            win_y_low  = binary_warped.shape[0] - (win+1)*window_h
            win_y_high = binary_warped.shape[0] - win*window_h
            win_xleft_low  = leftx_cur  - margin
            win_xleft_high = leftx_cur  + margin
            win_xright_low = rightx_cur - margin
            win_xright_high= rightx_cur + margin

            good_left  = ((nz_y>=win_y_low)&(nz_y<win_y_high)&
                          (nz_x>=win_xleft_low)&(nz_x<win_xleft_high)).nonzero()[0]
            good_right = ((nz_y>=win_y_low)&(nz_y<win_y_high)&
                          (nz_x>=win_xright_low)&(nz_x<win_xright_high)).nonzero()[0]
            left_inds.append(good_left); right_inds.append(good_right)

            if len(good_left) > minpix:  leftx_cur  = int(np.mean(nz_x[good_left]))
            if len(good_right)> minpix:  rightx_cur = int(np.mean(nz_x[good_right]))

        left_inds  = np.concatenate(left_inds)
        right_inds = np.concatenate(right_inds)
        if len(left_inds)<500 or len(right_inds)<500:    # 실패 조건
            return False, None, None, binary_warped

        left_fit  = np.polyfit(nz_y[left_inds],  nz_x[left_inds],  2)
        right_fit = np.polyfit(nz_y[right_inds], nz_x[right_inds], 2)
        return True, left_fit, right_fit, binary_warped

    # ───────── 3. 시각화 ─────────
    def draw_lane(self, orig, warp_bin, left_fit, right_fit):
        h, w = warp_bin.shape
        ploty = np.linspace(0, h-1, h)
        left_x  = np.polyval(left_fit,  ploty)
        right_x = np.polyval(right_fit, ploty)

        lane_img = np.zeros((h,w,3), dtype=np.uint8)
        pts_left  = np.array([np.transpose(np.vstack([left_x,  ploty]))])
        pts_right = np.array([np.flipud(np.transpose(np.vstack([right_x, ploty])))])
        pts = np.hstack((pts_left, pts_right))
        cv2.fillPoly(lane_img, np.int32([pts]), (0,255,0))
        newwarp = cv2.warpPerspective(lane_img, self.Minv, (orig.shape[1], orig.shape[0]))
        return cv2.addWeighted(orig, 1, newwarp, 0.3, 0)

# ───────── main ─────────
if __name__ == "__main__":
    rospy.init_node("Two_Line_detect_v2")
    LaneDetector()
    rospy.loginfo("LaneDetector v2 node started")
    rospy.spin()
