# ros2 action send_goal --feedback /fibonacci user_interface/Fibonacci "{step: 15 }"

import time

import rclpy
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sympy import fibonacci
from user_interface.action import Fibonacci


class Action_server(Node):
    def __init__(self):
        super().__init__("fibonacci_server")  # 노드 이름
        self.callback_group = ReentrantCallbackGroup()
        self.action_server = ActionServer(
            self, Fibonacci, "fibonacci", execute_callback=self.execute_callback
        )

    def execute_callback(self, goal_handle: ServerGoalHandle):
        # goal 을 받았을 때...
        request: Fibonacci.Goal = goal_handle.request
        self.get_logger().info(f"{request.step}")
        feedback = Fibonacci.Feedback()
        feedback.temp_seq = [0, 1]
        result = Fibonacci.Result()

        # feedback
        for i in range(1, request.step):
            feedback.temp_seq.append(feedback.temp_seq[i] + feedback.temp_seq[i - 1])
            goal_handle.publish_feedback(feedback)  # 피드백 보내기
            time.sleep(1)

        # result 보내기
        print(type(goal_handle))
        goal_handle.succeed()  # 완료 status 보내기
        # goal_handle.abort()  # aborted 상태로 보내기
        result.seq = feedback.temp_seq
        return result  # action 종료


def main():
    rclpy.init()
    node = Action_server()
    executor = MultiThreadedExecutor(num_threads=5)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        executor.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()