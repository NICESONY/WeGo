#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import speech_recognition as sr
import rospy
import pyaudio
import wave
import os
import sys
import rospy
from geometry_msgs.msg import Twist

CHUNK = 512
FORMAT = pyaudio.paInt16
CHANNELS = 2
RATE = 48000
RECORD_SECONDS = 3
WAVE_OUTPUT_FILENAME = "output.wav"

pub = rospy.Publisher('/cmd_vel',Twist, queue_size=10)
rospy.init_node('voice_ctr_node',anonymous=True)

def record():
    p = pyaudio.PyAudio()

    stream = p.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE,
                    input=True,
                    frames_per_buffer=CHUNK)

    print("recording...")

    frames = []

    for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
        data = stream.read(CHUNK)
        frames.append(data)

    print("done")

    stream.stop_stream()
    stream.close()
    p.terminate()

    wf = wave.open(WAVE_OUTPUT_FILENAME, 'wb')
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(p.get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(b''.join(frames))
    wf.close()

def detect_voice():
    r = sr.Recognizer()   
    test = sr.AudioFile(WAVE_OUTPUT_FILENAME) 
    with test as source:       
        audio = r.record(source)
    type(audio)
    c=r.recognize_sphinx(audio, language='en-US')
    return c 

def pub_cmd_msg(msg):
    rate = rospy.Rate(10)
    tick = 0
    while tick <= 30:       
        pub.publish(msg)
        tick = tick+1
        rate.sleep()

if __name__  == '__main__':
    while not input("1:start_recording\nq:quit\n:") == 'q':
        record()
        cmd = Twist()
        words = detect_voice()
        print(words)
        if "ahead" in words:
            cmd.linear.x = 0.5
            cmd.angular.z = 0.0
        if "back" in words:
            cmd.linear.x = -0.5
            cmd.angular.z = 0.0        
        if "right" in words:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5
        if "left" in words:
            cmd.linear.x = 0.0
            cmd.angular.z = -0.5
        pub_cmd_msg(cmd)
    exit(0)
