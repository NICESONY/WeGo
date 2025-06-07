## 리스트로 묶어서 tf를 발행하는 것이다. 그래서 tf 전용함수를 사용해서 하나로 발행을 진행할 수 있음
## sudo apt install ros-humble-tf-transformations

import rclpy as rp
import rclpy.logging
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import TransformStamped
from rclpy.node import  Node
from tf2_ros.transform_listener import TransformListener
# from tf2_ros import TRansformExcption
from turtlesim.srv import Spawn


## tf 가 여러가지 있을때 처음부터 끝까지 한버에 바로가는 각도와 거리 아는 ㄴ법
## 터틀심을 거북이를 열고 그녀석을 tf를 구독해ㅓ 따라가게하기

class FollowTultle(Node):
    def __init__(self):
        super().__init__("FollowTultle")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.on_timer)
        self.t = 0.0
        self.spawer = self.create_client(Spawn, "spawn")
        request = Spawn.Request()
        request.x = 3.0
        request.y = 3.0
        request.theta = 0.0
        self.result = self.spawer.call_async(request)
        print("done")


    def on_timer(self):
        try :
    
            t  = self.tf_buffer.lookup_transform("joint", 
                                                 "world", 
                                                 rclpy.time.Time())
            ## rclpy.time.Time() tf들 사이에서의 클락을 넘긴다 ??
        except Exception :
            self.get_logger().info("lookup failed") 
            return
        self.get_logger().info(f"{t.transform.translation.x}")
        self.get_logger().info(f"{t.transform.translation.y}")
        self.get_logger().info(f"{t.transform.translation.z}")

 

def main():
    rp.init()
    node = FollowTultle()
    try :
        rp.spin(node)
    except KeyboardInterrupt :
        node.destroy_node()

if __name__ =="__main__":
    main()