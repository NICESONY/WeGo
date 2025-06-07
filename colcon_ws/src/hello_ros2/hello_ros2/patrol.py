# ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
# ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=/home/aa/kuLimo/map.yaml
# initial pose 잡아서 amcl 활성화
# ros2 run hello_ros2 patrol

import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from nav2_msgs.action._follow_waypoints import FollowWaypoints_GetResult_Response
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
from rclpy.task import Future


class Action_client(Node):
    def __init__(self):
        super().__init__("fibonacci_client")  # 노드 이름
        self.action_client = ActionClient(self, FollowWaypoints, "follow_waypoints")
        self.future = Future()  # 메세지가 담길 오브젝트 future
        self.get_result_future = Future()  # 초기화
        # patrol_points 의 기준이 map tf 기준!!
        self.patrol_points = [(4.0, 0.0), (4.0, 1.0), (2.0, 2.5), (0.0, 1.0)]
        self.patrol_degree = [0, 90, 180, 90]
        self.patrol_index = 0
        self.goal = FollowWaypoints.Goal()
        self.go_next()

    def go_next(self):
        self.send_goal(
            self.patrol_points[self.patrol_index][0],
            self.patrol_points[self.patrol_index][1],
            self.patrol_degree[self.patrol_index],
        )
        self.patrol_index += 1
        if self.patrol_index >= len(self.patrol_points):
            self.patrol_index = 0

    def send_goal(self, x: float, y: float, theta: int):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        rad = math.radians(theta)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(rad / 2.0)
        pose.pose.orientation.w = math.cos(rad / 2.0)
        self.goal.poses.clear()  # type : ignore
        self.goal.poses.append(pose)  # type : ignore
        # 서버 접속
        while not self.action_client.wait_for_server(
            timeout_sec=1
        ):  # 서버 응답 대기 멈춤!
            self.get_logger().info("nav2 서버 접속중 ...")
        self.future: Future = self.action_client.send_goal_async(
            self.goal, feedback_callback=self.feedback_callback
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
        feedback: FollowWaypoints.Feedback = msg.feedback
        self.get_logger().info(f" 지금까지 처리 결과 seq{feedback.current_waypoint}")

    # result 를 처리하는 콜백 함수
    def done_callback(self, future: Future):
        result: FollowWaypoints_GetResult_Response = future.result()  # type : ignore
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"result: {result.result.missed_waypoints} 성공!!")
            self.go_next()
        if result.status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info(f"result: aborted 실패!!")


def main():
    rclpy.init()
    node = Action_client()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()


if __name__ == "__main__":
    main()