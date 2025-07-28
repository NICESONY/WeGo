#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
import speech_recognition as sr

class STTNode:
    def __init__(self):
        rospy.init_node('stt_node')
        self.pub = rospy.Publisher('/stt_topic', String, queue_size=10)

        # Recognizer 설정
        self.recognizer = sr.Recognizer()
        self.recognizer.dynamic_energy_threshold = True
        self.recognizer.energy_threshold = 300

        rospy.loginfo("STT node ready, adjusting for ambient noise...")

        # Microphone 열고 소음 학습한 뒤 루프 진입
        with sr.Microphone() as source:
            self.recognizer.adjust_for_ambient_noise(source, duration=2)
            rospy.loginfo("Listening for speech...")
            self.listen_loop(source)

    def listen_loop(self, source):
        while not rospy.is_shutdown():
            try:
                # 짧은 구간씩 캡처
                audio = self.recognizer.listen(source, timeout=5, phrase_time_limit=4)
                # 원시 바이트 데이터 획득
                buf = audio.get_raw_data()

                # 디버깅: 마지막 캡처 오디오 저장
                with open('/tmp/last_stt.wav', 'wb') as f:
                    f.write(buf)

                # Google Web Speech API 호출
                text = self.recognizer.recognize_google(audio, language='ko-KR')
                rospy.loginfo("STT recognized: %s", text)
                self.pub.publish(text)

            except sr.WaitTimeoutError:
                # 말이 없으면 재시도
                continue
            except sr.UnknownValueError:
                rospy.logwarn("Could not understand audio")
            except sr.RequestError as e:
                rospy.logerr("STT request failed: %s", e)

if __name__ == '__main__':
    try:
        STTNode()
    except rospy.ROSInterruptException:
        pass

