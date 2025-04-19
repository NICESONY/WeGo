#! /usr/bin/python3
import rospy
from std_msgs.msg import String
# ROS1노드 클래스가 존재하지 않음 import Node X

def main():
    rospy.init_node('hello', anonymous=True)
    pub = rospy.Publisher('message', String, queue_size=10)
    data = String()
    i=0
    data.data = f"hellom, ROS! netic {i}"
    rate = rospy.Rate(3)
    while not rospy.is_shutdown():
        pub.publish(data)
        print("hello, ROS1 noetic!!")
        rate.sleep()
        i += 1

if __name__ == "__main__":
    try :
        main()
    except rospy.ROSInterruptException :
        pass