import rclpy as rp
from rclpy.node import Node  # ros2 노드라는 클래스를 제공하므로 이걸로 클래스를 만들면 된다.
from std_srvs.srv import SetBool 
import time
"""
해당 코드는 Service server을 만들어서 RQT 그래프에서 TEST를 진행해 보는 실습 코드
먼저  Service/Service_Caller 실행 후 
이후 해당 위에 값을 변경 진행
"""
# 객체지향으로 만들기
class Service_server(Node):
    def __init__(self):
        super().__init__('simpleServiceServer')
        self.srv = self.create_service(SetBool,"simpleServiceServer", self.callback)
        self.bool = False
        self.cnt = 0
    
    def callback(self, 
                 request : SetBool.Request, 
                 response : SetBool.Response):
        self.get_logger().info(f"Service server is called : {self.cnt}")
        self.get_logger().info(f"bool importmation: {self.bool}")
        self.get_logger().info(f"변경 요청 값 : {request.data}")
        if request.data != self.bool :
            self.bool = not self.bool
            response.success = True
            response.message = f"{self.cnt}번째 요청 {self.bool}이 성공했습니다."
        else :
            response.success = False
            response.message = f"{self.cnt}번째 요청 {self.bool}이 실패했습니다."
        # self.bool = request.data
        # response.message = '변경이 성공했습니다'
        # response.success = True
        time.sleep(5)
        self.cnt += 1
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

