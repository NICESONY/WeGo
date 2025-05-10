import rclpy as rp
from rclpy.node import Node  # ros2 노드라는 클래스를 제공하므로 이걸로 클래스를 만들면 된다.
from std_srvs.srv import SetBool 
import time
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

"""
ROS2 에서 멀티쓰래드 구현
"""
# 객체지향으로 만들기
class Service_server2(Node):
    def __init__(self):
        super().__init__('simpleServiceServer2')

        self.callback_group = ReentrantCallbackGroup()
        self.srv = self.create_service(SetBool,
                                       "simpleService", 
                                       self.callback,
                                       callback_group = self.callback_group)
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
    node = Service_server2()
    executor = MultiThreadedExecutor(num_threads = 5)
    executor.add_node(node)
    # 자동적으로 threed가 만들어져서 진행된다.
    try :
        #rp.spin(node)
        executor.spin()
    except KeyboardInterrupt :
        executor.shutdown()
        node.destroy_node()
        rp.shutdown()
    


if __name__ == "__main__" :
    main()

