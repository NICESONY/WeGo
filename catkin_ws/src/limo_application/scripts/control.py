#! /usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Int16
from geometry_msgs.msg import Twist


class Control:
    def __init__(self):
        self.cmd_pub_ = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.error_sub_ = rospy.Subscriber('gap', Int16, self.errorCallback)
        self.stop_sub_ = rospy.Subscriber('stop', Bool, self.stopCallback)
        self.stop_flag_ = False

    def stopCallback(self, msg):
        self.stop_flag_ = msg.data
        
    def errorCallback(self, msg):
        cmd = Twist()
        cmd.linear.x = 0.3
        
        cmd.angular.z = 0.01 * msg.data
        # cmd.angular.z = 0.1 * msg.data()
        cmd.angular.z = min(max(-1.0, cmd.angular.z), 1.0)
        
        if self.stop_flag_:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            
        self.cmd_pub_.publish(cmd)
        
def main():
    rospy.init_node('control')
    c = Control()
    rospy.spin()
    
if __name__ == '__main__':
    main()