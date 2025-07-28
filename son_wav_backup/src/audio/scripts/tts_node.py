#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
import subprocess
from sound_play.libsoundplay import SoundClient

class TTSNode:
    def __init__(self):
        rospy.init_node('tts_node')
        self.soundhandle = SoundClient()

        # /tts_topic 토픽을 구독하면, msg.data에 담긴 텍스트를 음성으로 변환해 재생
        rospy.Subscriber('/tts_topic', String, self.callback)
        rospy.loginfo("TTS node ready, waiting for text on /tts_topic")
        rospy.spin()

    def callback(self, msg):
        text = msg.data
        rospy.loginfo("TTS -> “%s”" % text)

        # 1) espeak로 WAV 파일 생성 (임시)
        wav_path = '/tmp/tts_output.wav'
        cmd = ["espeak", "-v", "ko", "-s", "140", "-w", wav_path, text]
        subprocess.call(cmd)

        # 2) sound_play를 이용해 재생
        self.soundhandle.playWave(wav_path)

if __name__ == '__main__':
    try:
        TTSNode()
    except rospy.ROSInterruptException:
        pass

