#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rospy
import time
import numpy as np
from sensor_msgs.msg import Joy, LaserScan
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String as StringMsg
from simple_follower.msg import position as PositionMsg
from std_msgs.msg import Bool
		
class laserTracker:
	def __init__(self):
		self.lastScan = None
		self.winSize   = rospy.get_param('~winSize')
		self.deltaDist = rospy.get_param('~deltaDist')
		# 사용할 각도 범위 (rad) — 기본값 ±0.8rad
		self.angle_min = rospy.get_param('~angle_min', -0.8)
		self.angle_max = rospy.get_param('~angle_max', +0.8)

		self.scanSubscriber    = rospy.Subscriber('/scan', LaserScan, self.registerScan)
		self.positionPublisher = rospy.Publisher('/object_tracker/current_position', PositionMsg, queue_size=3)
		self.infoPublisher     = rospy.Publisher('/object_tracker/info', StringMsg, queue_size=3)
		self.laserTracker = rospy.Subscriber('/laser_tracker', Bool, self.laser_tracker)

		self.laser_tracker = False

	def laser_tracker(self, msg):
		self.laser_tracker = msg.data
		rospy.loginfo(f"[laser_tracker] mode = {self.laser_tracker}")



	def registerScan(self, scan_data):
		# --- 원본 거리 필터링 코드 (주석 처리) ---
		# ranges = np.array(scan_data.ranges)
		# ranges_list = []
		# for i in ranges :
		#     if i > 0.1:
		#         ranges_list.append(i)
		#     else:
		#         ranges_list.append(float('inf'))
		# ranges = ranges_list

		if not self.laser_tracker :
			return

		# 1) 각도 & 거리 필터링 적용
		angles = scan_data.angle_min + np.arange(len(scan_data.ranges)) * scan_data.angle_increment
		ranges = []
		for i, raw_dist in enumerate(scan_data.ranges):
			angle = angles[i]
			# 1. 각도 범위를 벗어나면 무시
			if angle < self.angle_min or angle > self.angle_max:
				ranges.append(float('inf'))
				continue
			# 2. 10cm 이하 또는 invalid 값은 노이즈로 간주
			if raw_dist <= 0.1 or np.isinf(raw_dist) or np.isnan(raw_dist):
				ranges.append(float('inf'))
				continue
			# 3. 나머지는 유효 거리
			ranges.append(raw_dist)

		# numpy array 로 변환
		ranges = np.array(ranges)
		# 거리 순 정렬
		sortedIndices = np.argsort(ranges)
		
		minDistanceID = None
		minDistance   = float('inf')		

		if self.lastScan is not None:
			for i in sortedIndices:
				tempMinDistance = ranges[i]
				windowIndex = np.clip([i-self.winSize, i+self.winSize+1], 0, len(self.lastScan))
				# self.lastScan 도 numpy array 이므로 슬라이스 결과는 numpy array
				window = self.lastScan[windowIndex[0]:windowIndex[1]]
				with np.errstate(invalid='ignore'):
					if np.any(np.abs(window - tempMinDistance) <= self.deltaDist):
						minDistanceID = i
						minDistance   = tempMinDistance
						break

		# 다음 스캔 비교를 위해 numpy array 로 저장
		self.lastScan = ranges  
		
		# 유효 객체 미검출 처리
		if minDistance > scan_data.range_max:
			rospy.logwarn('laser: no object found')
			self.infoPublisher.publish(StringMsg('laser:nothing found'))
		else:
			minDistanceAngle = scan_data.angle_min + minDistanceID * scan_data.angle_increment
			self.positionPublisher.publish(PositionMsg(minDistanceAngle, 42, minDistance))


if __name__ == '__main__':
	print('starting')
	rospy.init_node('laser_tracker')
	tracker = laserTracker()
	print('seems to do something')
	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		print('exception')
