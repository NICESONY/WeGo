#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import speech_recognition as sr
from geometry_msgs.msg import Twist

class VoiceControlKR:
    def __init__(self):
        rospy.init_node('voice_ctrl_kr', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Recognizer 설정
        self.recognizer = sr.Recognizer()
        self.recognizer.dynamic_energy_threshold = True
        self.recognizer.energy_threshold = 300

        # 마이크(기본 장치) 열고 소음 레벨 학습
        self.microphone = sr.Microphone()
        with self.microphone as source:
            rospy.loginfo("주변 소음 레벨 학습 중...")
            self.recognizer.adjust_for_ambient_noise(source, duration=1)
        rospy.loginfo("한국어 음성 명령 대기 중...")

        # 실시간 청취 시작
        self.listen_loop()

    def listen_loop(self):
        with self.microphone as source:
            while not rospy.is_shutdown():
                try:
                    # 최대 5초 대기, 한 문장당 최대 3초 녹음
                    audio = self.recognizer.listen(
                        source,
                        timeout=5,
                        phrase_time_limit=3
                    )
                except sr.WaitTimeoutError:
                    continue

                # 음성 인식
                try:
                    cmd_text = self.recognizer.recognize_google(
                        audio,
                        language='ko-KR'
                    )
                    rospy.loginfo("인식된 내용: %s", cmd_text)
                except sr.UnknownValueError:
                    rospy.logwarn("무슨 말인지 이해하지 못했습니다.")
                    continue
                except sr.RequestError as e:
                    rospy.logerr("STT API 요청 실패: %s", e)
                    continue

                # 명령어 매핑
                twist = Twist()
                if "앞" in cmd_text or "전진" in cmd_text:
                    twist.linear.x = 0.5
                elif "뒤" in cmd_text or "후진" in cmd_text:
                    twist.linear.x = -0.5
                elif "오른쪽" in cmd_text:
                    twist.angular.z = -0.5
                elif "왼쪽" in cmd_text:
                    twist.angular.z = 0.5
                else:
                    rospy.loginfo("이동 명령어가 감지되지 않았습니다.")
                    continue

                # 명령 발행 (0.5초간 유지)
                self.publish_cmd(twist, duration=0.5)

    def publish_cmd(self, twist_msg, duration=0.5):
        rate = rospy.Rate(10)  # 10Hz
        ticks = int(duration * 10)
        for _ in range(ticks):
            self.pub.publish(twist_msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        VoiceControlKR()
    except rospy.ROSInterruptException:
        pass

