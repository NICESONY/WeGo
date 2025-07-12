#! /usr/bin/env python
#-*- coding :UTF-8 -*-
import rospy
import serial
import time
import serial.tools.list_ports
from std_msgs.msg import UInt8
from std_msgs.msg import String
port_list = list(serial.tools.list_ports.comports())


class UpdownSerial:
    def __init__(self, name=None, baudrate=None):
        if name == None:
            name = "/dev/ttyUSB1"
        elif len(port_list) == 0:
            print('no serial')
            # else serial.
        if baudrate == None:
            baudrate = "115200"
        self.serial_port = serial.Serial(name, baudrate, timeout=0.1)
    def updown(self,UInt8):

        inputnum=UInt8.data
        rospy.loginfo(UInt8.data)

        if inputnum ==1:
            
            self.serial_port.write("#UP".encode('utf-8'))
            for index in range(10):
                
                if(self.serial_port.readline().encode('utf-8')== "CMD UP"):
                    print("CMD UP")
                    break
                else :
                    self.serial_port.write("#UP".encode('utf-8'))
                    pub.publish("UP")
                    # time.sleep(0.1)
            # rospy.loginfo(1)
                
        if inputnum ==2:
            self.serial_port.write("#DOWN".encode('utf-8'))
            # rospy.loginfo(2)
            
            for index in range(10):
                
                if(self.serial_port.readline().encode('utf-8')== "CMD DOWN"):
                    print("CMD DOWN")
                    break
                else :
                    self.serial_port.write("#DOWN".encode('utf-8'))
                    pub.publish("DOWN")
                    # time.sleep(0.1)
        
        if inputnum ==3:

            self.serial_port.write("#HOLD".encode('utf-8'))
            # rospy.loginfo(3)


if __name__=='__main__':
    rospy.init_node("listener_updown")
    c=UpdownSerial()
    pub = rospy.Publisher("chatter_temp", String, queue_size=1)
    sub = rospy.Subscriber("chatter_updown",UInt8,c.updown,queue_size=1)
    # sub = rospy.Subscriber("chatter_updown",UpdownSerial.updown,queue_size=10)
    rospy.spin() 
