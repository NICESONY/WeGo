#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String

def callback(msg):
    # 한글이 깨지지 않고 바로 출력됩니다.
    print msg.data

if __name__ == '__main__':
    rospy.init_node('stt_listener')
    rospy.Subscriber('/stt_topic', String, callback)
    rospy.spin()

