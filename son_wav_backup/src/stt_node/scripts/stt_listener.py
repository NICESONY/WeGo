#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String

class STTListener(object):
    def __init__(self):
        rospy.init_node('stt_listener', anonymous=True)
        rospy.loginfo("STT Listener started, waiting for messages on /stt_topic")
        self.sub = rospy.Subscriber('/stt_topic', String, self.callback)

    def callback(self, msg):
        # 1) print 로
        # print msg.data

        # 2) rospy.loginfo 로 (원하는 방식으로 하나만 쓰셔도 됩니다)
        rospy.loginfo("Received STT text: %s", msg.data)

def main() :
    node = STTListener()
    rospy.on_shutdown(node.shutdown)
    rospy.spin()

if __name__ == '__main__':
    main()
