import rclpy as rp
from rclpy.node import Node  # ros2 노드라는 클래스를 제공하므로 이걸로 클래스를 만들면 된다.
from std_srvs.srv import SetBool 


# 객체지향으로 만들기
class Service_server(Node):
    def __init__(self):
        super().__init__('simpleServiceServer')
        self.srv = self.create_service(SetBool,"simpleServiceServer", self.callback)
        self.bool = False

    
    def callback(self, 
                 request : SetBool.Request , 
                 response : SetBool.Response):
        self.get_logger().info("Service server is called")
        self.bool = request.data
        response.message = '변경이 성공했습니다'
        response.success = self.bool
        return response
    
    


def main():
    rp.init()
    node = Service_server()
    try :
        rp.spin(node)
    except KeyboardInterrupt :
        node.destroy_node()
    




if __name__ == "__main__" :
    main()

