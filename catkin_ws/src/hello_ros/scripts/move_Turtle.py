import rospy
# from std_msgs.msg import String
# ROS1노드 클래스가 존재하지 않음 import Node X
# 그래서 객체지향으로 코드를 작성하기 어려움 즉 ROS_02를 사용해야한다.
from geometry_msgs.msg import Twist

def main():
    rospy.init_node('hello', anonymous=True)
    pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
    data = Twist()
    i=0
    # data.data = f"hellom, ROS! netic {i}"
    
    data.linear.x = 2.0
    data.angular.z = 1.0

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish(data)
        # print("hello, ROS1 noetic!!")
        rate.sleep()
        # i += 1

if __name__ == "__main__":
    try :
        main()
    except rospy.ROSInterruptException :
        pass