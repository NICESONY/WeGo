import rclpy as rp
from rclpy.node import Node



class Hello(Node):
    def __init__(self) :
        super().__init__("hello_ros")
        self.create_timer(1, self.print_hello)

    def print_hello(self):
        print("hello ROS2 Humble !!")



def main():
    rp.init()
    node = Hello()

    try :
        rp.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        
    rp.spin(node)
    

if __name__ == "__main__" :
    main()