# ros2 action send_goal --feedback /fibonacci user_interface/Fibonacci "{step: 15 }"

import time

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from user_interface.action import Fibonacci
from user_interface.action._fibonacci import Fibonacci_GetResult_Response


class Action_client(Node):
    def __init__(self):
        super().__init__("fibonacci_client")  # 노드 이름
        self.callback_group = ReentrantCallbackGroup()
        self.action_client = ActionClient(self, Fibonacci, "fibonacci")
        self.future = Future()  # 메세지가 담길 오브젝트 future
        self.get_result_future = Future()  # 초기화
        # 서버 접속
        while not self.action_client.wait_for_server(
            timeout_sec=1
        ):  # 서버 응답 대기 멈춤!
            self.get_logger().info("피보나치 서버 접속중 ...")
        self.send_goal()

    def send_goal(self):
        goal = Fibonacci.Goal()
        goal.step = 8
        self.future: Future = self.action_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback
        )
        self.future.add_done_callback(self.goal_response_callback)

    # ros1 에 없는 체크 포인트( goal 이 접수 될 때)
    def goal_response_callback(self, future: Future):
        goal_handle: ClientGoalHandle = future.result()  # type : ignore
        if not goal_handle.accepted:
            self.get_logger().info("골이 접수 안 되었습니다.")
            return
        self.get_result_future: Future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.done_callback)

    def feedback_callback(self, msg):
        feedback: Fibonacci.Feedback = msg.feedback
        self.get_logger().info(f" 지금까지 처리 결과 seq{feedback.temp_seq}")

    # result 를 처리하는 콜백 함수
    def done_callback(self, future: Future):
        result: Fibonacci_GetResult_Response = future.result()  # type : ignore
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"result: {result.result.seq} 성공!!")
        if result.status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info(f"result: aborted 실패!!")


def main():
    rclpy.init()
    node = Action_client()
    executor = MultiThreadedExecutor(num_threads=5)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        executor.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()