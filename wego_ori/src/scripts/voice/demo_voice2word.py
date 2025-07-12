# -*- coding: utf-8 -*-
# /usr/bin/python

import speech_recognition as sr
import rospy
import sys

r = sr.Recognizer()  
file_voice = sys.argv[1]  
test = sr.AudioFile(file_voice) 
with test as source:       
    audio = r.record(source)
type(audio)
c=r.recognize_sphinx(audio, language='en-US')    
print(c)
