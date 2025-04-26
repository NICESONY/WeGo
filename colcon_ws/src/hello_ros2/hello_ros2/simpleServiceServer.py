import rclpy as rp
from rclpy.node import Node  # ros2 노드라는 클래스를 제공하므로 이걸로 클래스를 만들면 된다.
from std_msgs.srv import SetBool

# 객체지향으로 만들기
class Service_server(Node):
    pass


def main():
    rp.init()
    node = Service_server()
    try :
        rp.spin(node)
    except KeyboardInterrupt :
        node.destroy_node()
    
    



if __name__ == "__main__" :
    main()

