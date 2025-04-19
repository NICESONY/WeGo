import rclpy as rp
from rclpy.node import Node
from std_msgs.msg import String

class simple_sub(Node):
    def __init__(self):
        super().__init__("simple_sub")
        self.sub = self.create_subscription(String, "/message", self.simple_callback, 10)

    def simple_callback(self, msg : String) -> None:
        # print(msg.data)
        self.get_logger().info(msg.data)

    

# if you want to run this code
# ros2 topic pub --rate 1 /message std_msgs/msg/String "data 'hi'"


def main():
    rp.init()
    node = simple_sub()
    try :
        rp.spin(node)
    except KeyboardInterrupt :
        node.destroy_node()

if __name__ =="__main__":
    main()