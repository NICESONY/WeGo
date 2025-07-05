#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int16


class LineDetect:
    def __init__(self):
        self.img_sub_ = rospy.Subscriber(
            "camera/rgb/image_raw/compressed", CompressedImage, self.imgCallback
        )
        self.error_pub_ = rospy.Publisher("gap", Int16, queue_size=10)

        self.br_ = CvBridge()

        # 원래 ROS1 코드의 노란색 HSV 범위
        self.YELLOW_LOW_TH = np.array([0, 90, 100])
        self.YELLOW_HIGH_TH = np.array([60, 220, 255])

        self.center = 0.0
        self.find_line = False

    def imgCallback(self, msg):
        # 압축된 이미지를 BGR 형식으로 변환
        original_image = self.br_.compressed_imgmsg_to_cv2(msg, "bgr8")

        # ROI 설정 (하단 20% 영역)
        height, width = original_image.shape[:2]
        roi_image = original_image[int(height * 0.8) : height, 0:width]

        # HLS 색상 공간으로 변환 (원래 ROS1 코드 방식 유지)
        hls = cv2.cvtColor(roi_image, cv2.COLOR_BGR2HLS)
        mask = cv2.inRange(hls, self.YELLOW_LOW_TH, self.YELLOW_HIGH_TH)

        # 임계값 처리
        _, binary_mask = cv2.threshold(mask, 100, 255, cv2.THRESH_BINARY)

        # 디버깅용 마스크 이미지 표시
        debug_image = cv2.bitwise_and(roi_image, roi_image, mask=binary_mask)
        cv2.imshow("masked_image", debug_image)

        error = Int16()

        # 마지막 행에서 노란색 픽셀 찾기
        last_row = binary_mask[-1, :]
        line_indices = []
        for i, pixel in enumerate(last_row):
            if pixel > 0:  # 노란색 픽셀
                line_indices.append(i)

        if line_indices:
            # 선의 최소/최대 값으로 중심 계산
            max_v = max(line_indices)
            min_v = min(line_indices)
            self.center = min_v
            print(f"Max: {max_v}, Min: {min_v}, Center: {self.center}")

            # 중심점 기준 에러 계산 (이미지 중심에서 얼마나 벗어났는지)
            image_center = width // 2
            error.data = int(
                image_center - self.center
            )  # 오른쪽으로 벗어나면 양수, 왼쪽은 음수
            self.find_line = True

            # 디버깅용 중심점 표시
            cx = int(self.center)
            cy = int(height * 0.9)  # ROI 내 중심점 y 좌표
            cv2.circle(original_image, (cx, cy), 10, (255, 0, 0), -1)
        else:
            self.find_line = False
            error.data = 0  # 선을 찾지 못하면 에러 0

        self.error_pub_.publish(error)

        # 디버깅 이미지 표시
        cv2.imshow("original_image", original_image)
        cv2.imshow("roi_image", roi_image)
        cv2.waitKey(3)


def main():
    rospy.init_node("line_detect")
    dl = LineDetect()
    rospy.spin()


if __name__ == "__main__":
    main()