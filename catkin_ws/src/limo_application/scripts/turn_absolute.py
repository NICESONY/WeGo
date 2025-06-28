#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import math

class Turn_absolute:
    
    def turn(self):
        
        cmd = Twist()
        cmd.linear.x = 0.0
        gap = self.goal_yaw - self.yaw
        
        while math.pi < gap:
            gap -= 2 * math.pi
        while -math.pi > gap:
            gap += 2 * math.pi
        print(gap)
        
        if gap > 0:
            cmd.angular.z = 0.3
        else:
            cmd.angular.z = -0.3
            
        if abs(gap) < 0.05:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            print("goal reached")
        
        self.cmd_pub.publish(cmd)
        
    
    def __init__(self):
        self.sub_ = rospy.Subscriber('imu',
                                     Imu, 
                                     self.imuCallback)
        
        self.cmd_pub =rospy.Publisher('cmd_vel', 
                                      Twist, 
                                      queue_size=10)
        
        self.yaw = 0.0
        self.goal_yaw = 2.0
        
            
        
    def imuCallback(self, msg): #오일러 공식 -> 쿼터니언 공식으로 변환
        (self.roll, self.pitch, self.yaw) = self.euler_from_quaternion(msg.orientation.x,
                                                                       msg.orientation.y,
                                                                       msg.orientation.z,
                                                                       msg.orientation.w)
    
        
    import math
    def euler_from_quaternion(self,x, y, z, w):
            """
            Convert a quaternion into euler angles (roll, pitch, yaw)
            roll is rotation around x in radians (counterclockwise)
            pitch is rotation around y in radians (counterclockwise)
            yaw is rotation around z in radians (counterclockwise)
            """
            t0 = +2.0 * (w * x + y * z)
            t1 = +1.0 - 2.0 * (x * x + y * y)
            roll_x = math.atan2(t0, t1)
        
            t2 = +2.0 * (w * y - z * x)
            t2 = +1.0 if t2 > +1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            pitch_y = math.asin(t2)
        
            t3 = +2.0 * (w * z + x * y)
            t4 = +1.0 - 2.0 * (y * y + z * z)
            yaw_z = math.atan2(t3, t4)
        
            return roll_x, pitch_y, yaw_z # in radians
        
    
def main():
    rospy.init_node('turn_absolute')
    ta = Turn_absolute()
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        ta.turn()
        rate.sleep()
    
if __name__ == '__main__':
    main()