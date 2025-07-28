#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
import speech_recognition as sr

class STTNode:
    def __init__(self):
        rospy.init_node('stt_node')
        self.pub = rospy.Publisher('/stt_topic', String, queue_size=10)

        # (디버그) 사용 가능한 마이크 리스트 출력
        mic_list = sr.Microphone.list_microphone_names()
        for idx, name in enumerate(mic_list):
            rospy.loginfo("Mic %d: %s", idx, name)

        # USB 마이크 (hw:2,0) → 인덱스 11
        mic_index = 11
        rospy.loginfo("Using Mic index %d: %s", mic_index, mic_list[mic_index])

        # sample_rate 파라미터 제거하여 OS 디폴트 사용
        self.microphone = sr.Microphone(device_index=mic_index)
        self.recognizer = sr.Recognizer()

        rospy.loginfo("STT node ready, say something on the mic...")
        self.listen_loop()

    def listen_loop(self):
        with self.microphone as source:
            # 주변 소음 레벨 측정
            self.recognizer.adjust_for_ambient_noise(source, duration=1)
            while not rospy.is_shutdown():
                rospy.loginfo(">> listening...")
                try:
                    audio = self.recognizer.listen(source, timeout=5)
                    rospy.loginfo(">> audio captured, processing...")
                    text = self.recognizer.recognize_google(audio, language='ko-KR')

                    # 로그에 남길 때는 UTF-8로 인코딩
                    try:
                        rospy.loginfo("STT recognized: %s", text.encode('utf-8'))
                    except Exception:
                        rospy.loginfo("STT recognized (non-ASCII text)")

                    # 퍼블리시할 때는 원문 그대로 사용
                    self.pub.publish(text)

                except sr.WaitTimeoutError:
                    # 타임아웃 시 재청취
                    continue
                except sr.UnknownValueError:
                    rospy.logwarn("Could not understand audio")
                except sr.RequestError as e:
                    rospy.logerr("API request failed: %s", e)

if __name__ == '__main__':
    try:
        STTNode()
    except rospy.ROSInterruptException:
        pass

