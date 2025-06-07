## 리스트로 묶어서 tf를 발행하는 것이다. 그래서 tf 전용함수를 사용해서 하나로 발행을 진행할 수 있음
## sudo apt install ros-humble-tf-transformations

import rclpy as rp
import rclpy.logging
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rclpy.node import  Node
from tf_transformations import quaternion_from_euler


class DynamicFramePublisher(Node):
    def __init__(self):
        super().__init__("dynamic_tf2_broadcaster")
        self.create_timer(1/30, self.pub_cv)
        self.tf_br = TransformBroadcaster(self) ## 본인의 객체만 넘긴다.
        self.t = 0.0

    def pub_cv(self):
        t  = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world' # 전부 상대적인 값이다 parent 와 child Tf 간의
        t.child_frame_id = "map"

        t.transform.translation.x = 1.0
        t.transform.translation.y = 1.0
        t.transform.translation.z = 0.0
        quat = quaternion_from_euler(0, 0, self.t) # r y  p 
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]



        t2 = TransformStamped()

        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = 'map' # 전부 상대적인 값이다 parent 와 child Tf 간의
        t2.child_frame_id = "joint"

        t2.transform.translation.x = 3.0
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.0
        t2.transform.rotation.x = 0.0
        t2.transform.rotation.y = 0.0
        t2.transform.rotation.z = 0.0
        t2.transform.rotation.w = 1.0   
        self.t += 1 / 60
        self.tf_br.sendTransform(t)
        self.tf_br.sendTransform(t2)


def main():
    rp.init()
    node = DynamicFramePublisher()
    try :
        rp.spin(node)
    except KeyboardInterrupt :
        node.destroy_node()

if __name__ =="__main__":
    main()