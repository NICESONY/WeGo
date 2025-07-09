#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def image_cb(msg):
    cv_img = CvBridge().imgmsg_to_cv2(msg, "bgr8")
    # TODO: cv_img 처리
    cv2.imshow("sim_cam", cv_img)
    cv2.waitKey(1)

if __name__ == "__main__":
    rospy.init_node("sim_cam_sub")
    rospy.Subscriber("/limo/color/image_raw", Image, image_cb, queue_size=1)
    rospy.spin()
