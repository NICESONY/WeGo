import rclpy as rp
from rclpy.node import Node



class Hello(Node):
    def __init__(self) :
        super().__init__("hello_ros")

def print_hello():
    print("hello ROS2 Humble")

def main():
    rp.init()
    node = Hello()
    node.create_timer(1, print_hello)
    try :
        rp.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
    rp.spin(node)
    

if __name__ == "__main__" :
    main()