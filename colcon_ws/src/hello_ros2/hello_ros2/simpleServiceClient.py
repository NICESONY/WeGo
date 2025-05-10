import rclpy as rp
from rclpy.node import Node  # ros2 노드라는 클래스를 제공하므로 이걸로 클래스를 만들면 된다.
from std_srvs.srv import SetBool 
import time
from asyncio import Future

"""
client code
"""
# 객체지향으로 만들기
class Service_Client(Node):
    def __init__(self):
        super().__init__('simpleServiceClient')
        self.client = self.create_client(SetBool,"simpleService")
        while not self.client.wait_for_service(timeout_sec=1.0) :
            self.get_logger().info("service not available")
        self.create_timer(1, self.update)
        self.create_timer(2, self.send_request)
        self.bool = False
        self.cnt = 0
        self.request = SetBool.Request()
        self.future = Future()


    def update(self):
        self.get_logger().info("main Thread is running")


    def send_request(self):
        self.get_logger().info(f"{self.cnt}번째 요청")
        self.request.data = not self.request.data
        self.future =  self.client.call_async(self.request) # 비동기로 보낼 수 있음
        self.future.add_done_callback(self.done_callback)
        self.cnt += 1

    def done_callback(self, future):
        response : SetBool.Response = future.result()
        self.get_logger().info(f"{response.message}")
        self.get_logger().info(f"{response.success}")


def main():
    rp.init()
    node = Service_Client()
    try :
        rp.spin(node)
    except KeyboardInterrupt :
        node.destroy_node()
    


if __name__ == "__main__" :
    main()

