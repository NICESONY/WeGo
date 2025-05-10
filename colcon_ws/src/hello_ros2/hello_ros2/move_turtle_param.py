import rclpy as rp
from rclpy.node import Node
from geometry_msgs.msg import Twist


"""
# move turtle.py -> 파마리터를 설정 각 속도 움직을 외부에서 변화
# 위 파라미터도  yaml에 넣어서 작동

## 파라미터 yaml 만들기

1. 실행
ros2 run hello_ros2 move_turtle_param --ros-args -p lin_inc:=0.02 -p ang_vel:=2.0 -p period:=0.05 &


2. 다른 터미널에서 dump
ros2 param dump /move_turtle_param > move_turtle_param.yaml

"""
class MoveTurtle(Node):
    def __init__(self):
        super().__init__('move_turtle_param')

        # 파라미터 선언
        self.declare_parameter('lin_inc', 0.005)
        self.declare_parameter('ang_vel', 1.0)
        self.declare_parameter('init_lin', 0.0)
        self.declare_parameter('period', 0.1)

        # 파라미터 값 읽기 
        self.lin_inc = self.get_parameter('lin_inc').value
        self.ang_vel = self.get_parameter('ang_vel').value
        self.cur_lin = self.get_parameter('init_lin').value
        period       = self.get_parameter('period').value

        # 퍼블리셔 & 타이머 
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_timer(period, self.pub_turtle_callback)

    # 주기적 퍼블리시 
    def pub_turtle_callback(self):
        msg = Twist()
        msg.angular.z = self.ang_vel
        msg.linear.x  = self.cur_lin
        self.pub.publish(msg)
        self.cur_lin += self.lin_inc

def main():
    rp.init()
    node = MoveTurtle()
    try :
        rp.spin(node)
    except KeyboardInterrupt :
        rp.shutdown()
        node.destroy_node()

        
if __name__ == '__main__':
    main()
