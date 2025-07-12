#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import cv2
import math
import os
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image



# 高斯模糊窗口大小
kernel_size = 3

# 边缘检测阈值
low_threshold = 50
high_threshold = 150

# 感兴趣区域梯形设置
trap_bottom_width = 0.85  # 底部宽度占比
trap_top_width = 0.07  # 顶部宽度占比
trap_height = 0.4  # 高度占比

# 霍夫变换参数
rho = 2 # 距离分辨率（以霍夫网格的像素为单位）
theta = 1 * np.pi/180 # 霍夫网格的角度分辨率（弧度）
threshold = 15	 # 最小阈值（霍夫网格单元中的交点）
min_line_length = 10 #构成线条的最小像素数
max_line_gap = 20	# 可连接线段之间的最大像素间距

#相机参数
fx=604.7211303710938#x轴焦距
fy=603.2431640625#y轴焦距
width=1280#640
height=720#480
cam_height=1.0#摄像头安装高度
cam_center=0.0#摄像头距离中间位置

#ROS参数
img=0
pub_img=0


def grayscale(img):
	"""
    将图片转换为灰度图
    """
	return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	
def canny(img, low_threshold, high_threshold):
	"""
    边缘检测
    """
	return cv2.Canny(img, low_threshold, high_threshold)

def gaussian_blur(img, kernel_size):
	"""
    高斯滤波，模糊
    """
	return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)

def region_of_interest(img, vertices):
	"""
    框选感兴趣区域，即车道线所在的梯形区域
    """
	#初始化空白图片
	mask = np.zeros_like(img)   
	
	#根据输入图像定义3通道或1通道颜色以填充遮罩
	if len(img.shape) > 2:
		channel_count = img.shape[2]  # i.e. 3 or 4 depending on your image
		ignore_mask_color = (255,) * channel_count
	else:
		ignore_mask_color = 255
		
	#填充由定义的多边形内的像素
	cv2.fillPoly(mask, vertices, ignore_mask_color)
	
	#与原图片进行与操作
	masked_image = cv2.bitwise_and(img, mask)
	return masked_image

def draw_lines(img, lines, color=[255, 0, 0], thickness=10):
	"""
    绘制车道线
    坡度（（y2-y1）/（x2-x1））决定哪些线段是左侧的一部分
    此函数用于使用“color”和“thickness”绘制“lines”。
	"""
	# 判断参数是否正确
	if lines is None:
		return
	if len(lines) == 0:
		return
	draw_right = True
	draw_left = True
	
	#查找所有直线的坡度
	#但只关心abs（slope）>坡度阈值的线
	slope_threshold = 0.5
	slopes = []
	new_lines = []
	for line in lines:
		x1, y1, x2, y2 = line[0]  # line = [[x1, y1, x2, y2]]
		
		# 计算坡度
		if x2 - x1 == 0.:  # 避免整数除以0
			slope = 999.  # 坡度最大值
		else:
			slope = 1.0*(y2 - y1) / (x2 - x1)
			
		#基于坡度过滤线
		if abs(slope) > slope_threshold:
			slopes.append(slope)
			new_lines.append(line)
		
	lines = new_lines
    
    
	#将线拆分为右线和左线，表示右车道线和左车道线
	#右/左车道线必须具有正/负坡度，并且位于图像的右/左半部分
	right_lines = []
	left_lines = []
	for i, line in enumerate(lines):
		x1, y1, x2, y2 = line[0]
		img_x_center = img.shape[1] / 2  # x coordinate of center of image
		if slopes[i] > 0 and x1 > img_x_center and x2 > img_x_center:
			right_lines.append(line)
		elif slopes[i] < 0 and x1 < img_x_center and x2 < img_x_center:
			left_lines.append(line)
			
	# 线性回归，找到左右车道线的最佳拟合线
	# 右边车道线
	right_lines_x = []
	right_lines_y = []
	
	for line in right_lines:
		x1, y1, x2, y2 = line[0]
		
		right_lines_x.append(x1)
		right_lines_x.append(x2)
		
		right_lines_y.append(y1)
		right_lines_y.append(y2)
		
	if len(right_lines_x) > 0:
		right_m, right_b = np.polyfit(right_lines_x, right_lines_y, 1)  # y = m*x + b
	else:
		right_m, right_b = 1, 1
		draw_right = False
		
	# 左边车道线
	left_lines_x = []
	left_lines_y = []
	
	for line in left_lines:
		x1, y1, x2, y2 = line[0]
		
		left_lines_x.append(x1)
		left_lines_x.append(x2)
		
		left_lines_y.append(y1)
		left_lines_y.append(y2)
		
	if len(left_lines_x) > 0:
		left_m, left_b = np.polyfit(left_lines_x, left_lines_y, 1)  # y = m*x + b
	else:
		left_m, left_b = 1, 1
		draw_left = False
	
	# 为左右线找到两个端点，用于绘制线
	# y = m*x + b --> x = (y - b)/m
	y1 = img.shape[0]
	y2 = img.shape[0] * (1 - trap_height)
	
	right_x1 = (y1 - right_b) / right_m
	right_x2 = (y2 - right_b) / right_m
	
	left_x1 = (y1 - left_b) / left_m
	left_x2 = (y2 - left_b) / left_m
	
	# 将float转换为int类型
	y1 = int(y1)
	y2 = int(y2)
	right_x1 = int(right_x1)
	right_x2 = int(right_x2)
	left_x1 = int(left_x1)
	left_x2 = int(left_x2)
	# 绘制左右车道线
	if draw_right:
		cv2.line(img, (right_x1, y1), (right_x2, y2), color, thickness)
	if draw_left:
		cv2.line(img, (left_x1, y1), (left_x2, y2), color, thickness)
	if draw_right and draw_left:
		left,right=calc_distance_from_center(0.5*width-1.0*left_x1, 1.0*right_x1-0.5*width)
		center=(left-right)/2-cam_center
		text="left:"+str(round(left, 2))+"  right:"+str(round(right, 2))+"  "+str(round(center, 2))+"  m off center"
		img = cv2.putText(img, text, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 2)
	
    
def hough_lines(img, rho, theta, threshold, min_line_len, max_line_gap):
	"""
	利用霍夫变换寻找直线
	"""
	lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
	line_img = np.zeros((img.shape[0],img.shape[1], 3), dtype=np.uint8)  # 3-channel RGB image
	draw_lines(line_img, lines)
	return line_img


def weighted_img(img, initial_img, a=0.8, b=1., c=0.):
	"""
	用于将黄线和白线区域图像合并
	"""
	return cv2.addWeighted(initial_img, a, img, b, c)


def filter_colors(image):
	"""
	将图像过滤，剩余白色部分以及黄色部分
	"""
	# 过滤白色
	white_threshold = 200 #130
	lower_white = np.array([white_threshold, white_threshold, white_threshold])
	upper_white = np.array([255, 255, 255])
	white_mask = cv2.inRange(image, lower_white, upper_white)
	white_image = cv2.bitwise_and(image, image, mask=white_mask)

	# 过滤黄色
	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	lower_yellow = np.array([90,100,100])
	upper_yellow = np.array([110,255,255])
	yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
	yellow_image = cv2.bitwise_and(image, image, mask=yellow_mask)

	# 合并图片
	image2 = cv2.addWeighted(white_image, 1., yellow_image, 1., 0.)

	return image2

def annotate_image(image_in):
	""" 
    对一张图片进行处理
    """
	# 过滤颜色
	image = filter_colors(image_in)

	# 将图像转换为灰度图
	gray = grayscale(image)

	# 高斯模糊处理
	blur_gray = gaussian_blur(gray, kernel_size)

	# 边缘检测
	edges = canny(blur_gray, low_threshold, high_threshold)

	# 设置感兴趣区域
	imshape = image.shape
	vertices = np.array([[\
		((imshape[1] * (1 - trap_bottom_width)) // 2, imshape[0]),\
		((imshape[1] * (1 - trap_top_width)) // 2, imshape[0] - imshape[0] * trap_height),\
		(imshape[1] - (imshape[1] * (1 - trap_top_width)) // 2, imshape[0] - imshape[0] * trap_height),\
		(imshape[1] - (imshape[1] * (1 - trap_bottom_width)) // 2, imshape[0])]]\
		, dtype=np.int32)
	masked_edges = region_of_interest(edges, vertices)

	# 霍夫直线检测
	line_image = hough_lines(masked_edges, rho, theta, threshold, min_line_length, max_line_gap)
	

	# 绘制车道线
	initial_image = image_in.astype('uint8')
	annotated_image = weighted_img(line_image, initial_image)
	
	return annotated_image

def calc_distance_from_center(left,right):
    z_px=math.sqrt(fx*fx+left*left)
    z=z_px/(0.5*height/cam_height)
    left=1.0*left/(z_px/z)
    
    z_px=math.sqrt(fx*fx+right*right)
    z=z_px/(0.5*height/cam_height)
    right=1.0*right/(z_px/z)
    return left,right

def callback(imgmsg):
	bridge=CvBridge()
	img=bridge.imgmsg_to_cv2(imgmsg, "rgb8")
	height=img.shape[0]
	width=img.shape[1]
	img=annotate_image(img)
	pub_img.publish(bridge.cv2_to_imgmsg(img, "rgb8"))


if __name__ == '__main__':
	rospy.init_node('lane_detection',anonymous=True)
	fx=rospy.get_param('~fx')
	fy=rospy.get_param('~fy')
	cam_height=rospy.get_param('~cam_height')
	cam_center=rospy.get_param('~cam_center')
	rospy.Subscriber("/camera/color/image_raw",Image,callback)
	pub_img=rospy.Publisher('/lane_detection/image_raw', Image, queue_size=1) 
	#pub_distance=rospy.Publisher('/lane_detection/off_center', Float32, queue_size=1) 

	rospy.spin()

