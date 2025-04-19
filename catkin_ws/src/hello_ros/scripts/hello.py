#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

class HelloNode:
    def __init__(self):
        # 퍼블리셔 선언
        self.pub = rospy.Publisher('message', String, queue_size=10)
        self.i = 0
        # 0.33초 간격으로 콜백 실행 (Duration으로 감싸야 합니다)
        rospy.Timer(rospy.Duration(0.33), self.print_hello)

    def print_hello(self, event):
        msg = String()
        msg.data = f"hello, ROS! noetic {self.i}"
        self.i += 1
        self.pub.publish(msg)
        rospy.loginfo(f"Published: {msg.data}")

def main():
    rospy.init_node('hello_node', anonymous=True)
    node = HelloNode()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
