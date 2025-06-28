#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

class LimoControl:
    def __init__(self):
        self.pub_ = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        
        self.msg = Twist()
        self.msg.linear.x = 0.3
        self.msg.angular.z = 0.3

    def publishData(self):
        self.pub_.publish(self.msg)

def main():
    rospy.init_node('Limo_control')
    tc = LimoControl()
    
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        tc.publishData()
        rate.sleep()
    
if __name__ == '__main__':
    main()