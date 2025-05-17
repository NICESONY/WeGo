import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from user_interface.msg import MyTopic


class MyTopicPub(Node):
    def __init__(self):
        super().__init__('my_Topic_Pub')
        # INFO 레벨 강제
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)

        self.pub = self.create_publisher(MyTopic, '/message', 10)
        self.timer = self.create_timer(0.1, self.pub_message)
        self.publish_count = 0

    def pub_message(self):
        msg = MyTopic()
        msg.first_name = 'hello ros2'
        msg.age = 15
        msg.score = 20
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'내가 만든 토픽입니다. {self.publish_count}'

        self.pub.publish(msg)
        self.publish_count += 1

        # ① rclpy 로거 (rosout‧터미널 모두 출력)
        self.get_logger().info(f'Published #{self.publish_count}')

        # ② 표준 출력 (터미널만 출력)
        print(f'[PRINT] Published #{self.publish_count}', flush=True)


def main():
    rclpy.init()
    node = MyTopicPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
