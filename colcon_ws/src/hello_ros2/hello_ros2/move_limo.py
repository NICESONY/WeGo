import rclpy as rp
from rclpy.node import Node
from geometry_msgs.msg import Twist


class move_turtle(Node) :
    def __init__(self) :
        super().__init__("move_limo")
        self.create_timer(0.1, self.pub_turtle)
        self.pub = self.create_publisher(Twist, "/cmd_vel" , 10)
        self.count = 0
        self.val = 0.0


    def pub_turtle(self):
        msg = Twist() # 클래스 객체로 해야함
        msg.angular.z= 1.0  ## python type 캐스팅이 자유롭다.
        # msg.linear.x = 2.0
        msg.linear.x = self.val  ## 하지만 dds로 넘길때 type check가 되어야한다.
        self.pub.publish(msg)
        self.val += 0.005



def main() :
    rp.init()
    node = move_turtle()

    try :
        rp.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()

    rp.spin(node)
    

if __name__ == "__main__" :
    main()