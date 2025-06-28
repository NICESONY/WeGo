#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import math

class Stop:
    def __init__(self):
        self.laser_sub_ = rospy.Subscriber('scan',
                                           LaserScan,
                                           self.laserCallback)
        
        self.stop_pub_ = rospy.Publisher('stop', Bool, queue_size=10)
    
    def laserCallback(self, msg):
        stop_flag = Bool()
        stop_flag.data = False
        
        # x = ranges[index]cos(theta)
        # y = ranges[index]sin(theta)
        # theta = angle_min + index * angle_increment
        for index, data in enumerate(msg.ranges):
            theta = msg.angle_min + index * msg.angle_increment
            cx = data * math.cos(theta)
            cy = data * math.sin(theta)
            
            if 0.01 < cx < 0.2 and -0.1 < cy <0.1:
                stop_flag.data = True
                break         
        self.stop_pub_.publish(stop_flag)
    
def main():
    rospy.init_node('stop')
    stop = Stop()
    rospy.spin()
    
if __name__ == '__main__':
    main()