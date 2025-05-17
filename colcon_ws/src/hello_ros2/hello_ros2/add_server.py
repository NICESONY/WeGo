import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from user_interface.srv import AddTwoInts


class Service_server(Node):
    def __init__(self):
        super().__init__("add_service_server")  # 노드 이름
        self.callback_group = ReentrantCallbackGroup()
        self.create_service(
            AddTwoInts,
            "add_ints",
            self.setBool_callback,
            callback_group=self.callback_group,
        )

    def setBool_callback(
        self, request: AddTwoInts.Request, response: AddTwoInts.Response
    ):
        # 서버의 작동 알고리즘...
        self.get_logger().info(f"{request.header.stamp} 시간")
        self.get_logger().info(f"a : {request.a}")
        self.get_logger().info(f"b : {request.b}")
        response.result = request.a + request.b
        response.success = True
        response.message = "a 와 b 의 값을 더해서 반환 했다!"
        time.sleep(5)
        return response


def main():
    rclpy.init()
    node = Service_server()
    executor = MultiThreadedExecutor(num_threads=5)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        executor.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()