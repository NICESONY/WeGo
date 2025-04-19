import rclpy as rp
from rclpy.node import Node
from std_msgs.msg import String

class simple_pub(Node):
    def __init__(self):
        super().__init__("simple_pub")
        self.pub = self.create_publisher(String, "/message",  10)
        self.create_timer(0.1, self.pub_message)

    def pub_message(self):
        msg = String()
        msg.data = "hello ros2"
        self.pub.publish(msg)

def main():
    rp.init()
    node = simple_pub()
    try :
        rp.spin(node)
    except KeyboardInterrupt :
        node.destroy_node()

if __name__ =="__main__":
    main()