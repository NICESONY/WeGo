# ros2 topic echo /map --once
# ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom - tf 임이 발행
# ros2 run hello_ros2 publish_map

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node

"""
opencv 에서의 메트릭스라는 라이브러를  사용해서 진행하는 부분 질의 -> 직접해보라는 것을 추천
해당 픽셀의 정보를 바탕으로 주위의 장애물을 확인하고 회피하도록 만들기 필요함.
이에 코드를 실행하고 rviz2 실행 GOGO
"""

class PublishMap(Node) :
    def __init__(self) :
        super().__init__("publish_map")  # node name
        self.create_timer(0.001, self.pub_cb)
        self.pub = self.create_publisher(OccupancyGrid, "/map", 100)
        self.msg = OccupancyGrid()
        self.msg.header.frame_id = "odom"
        self.msg.info.resolution = 0.1
        self.msg.info.width = 200
        self.msg.info.height = 100
        self.msg.info.origin.position.x = 0.0
        self.msg.info.origin.position.y = 0.0
        self.msg.info.origin.position.z = 0.0
        self.msg.info.origin.orientation.x = 0.0
        self.msg.info.origin.orientation.y = 0.0
        self.msg.info.origin.orientation.z = 0.0
        self.msg.info.origin.orientation.w = 1.0

        self.msg.data = [100 for _ in range(10_000)]
        self.msg.data.extend([0 for _ in range(10_000)])

        self.count = 0
        self.row = 0

    def pub_cb(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()

        index = self.count + (self.msg.info.width * self.row)
        if self.msg.data[index] == -1:
            self.msg.data[index] = 100
        else:
            self.msg.data[index] = -1

        self.count += 1
        if self.count >= self.msg.info.width:
            self.count = 0
            self.row += 1
        if self.row >= self.msg.info.height:
            self.row = 0

        self.pub.publish(self.msg)


def main():
    rclpy.init()
    node = PublishMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()


if __name__ == "__main__":
    main()