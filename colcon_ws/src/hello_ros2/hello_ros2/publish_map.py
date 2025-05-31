#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion


"""
slam에서 충분히 만족하지 못할때 해당 코드를 통해서 map을 만들고 map으로 경로를 이동할 수 있음.

"""
class SimpleMapPublisher(Node):
    def __init__(self):
        super().__init__('simple_map_publisher')
        self.publisher = self.create_publisher(OccupancyGrid, '/map', 10)
        self.timer = self.create_timer(1.0, self.publish_map)  # 1초 주기

        # 예시: 10×10 맵, 해상도 0.5m = 5×5 미터 공간
        self.map_width = 10
        self.map_height = 10
        self.resolution = 0.5  # 0.5m

        # 맵 메타데이터 설정
        self.map_info = MapMetaData()
        self.map_info.map_load_time = self.get_clock().now().to_msg()
        self.map_info.resolution = self.resolution
        self.map_info.width = self.map_width
        self.map_info.height = self.map_height

        # origin: 맵 배열의 (0,0)이 실제 세계에서 (-2.5, -2.5)에 위치
        self.map_info.origin = Pose()
        self.map_info.origin.position = Point(x=-2.5, y=-2.5, z=0.0)
        self.map_info.origin.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # 간단한 맵 배열 생성: 중앙을 장애물(100)로, 나머지는 비어있음(0)
        self.data = []
        for y in range(self.map_height):
            for x in range(self.map_width):
                if x == self.map_width // 2 and y == self.map_height // 2:
                    self.data.append(100)  # 중앙 셀을 벽(occupancy 100)으로 설정
                else:
                    self.data.append(0)    # 나머지는 빈 공간(occupancy 0)

    def publish_map(self):
        msg = OccupancyGrid()
        # 1) Header 설정
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        # 2) MapMetaData 설정
        msg.info = self.map_info

        # 3) 데이터 배열 설정
        msg.data = self.data

        # 4) 퍼블리시
        self.publisher.publish(msg)
        self.get_logger().info(f'Published map: {self.map_width}×{self.map_height}, resolution={self.resolution}')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
