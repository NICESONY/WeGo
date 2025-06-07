# sudo apt install ros-humble-tf-transformations
# sudo apt install ros-humble-tf-transformations
# ros2 run turtlesim turtlesim_node
# ros2 run hello_ros2 follow_turtlesim
# rviz2 -> tf 확인
# ros2 run turtlesim turtle_teleop_key

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from rclpy import time
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from turtlesim.msg import Pose
from turtlesim.srv import Spawn

# from tf2_ros import TransformException


class dynamic_tf_turtlesim(Node):
    def __init__(self):
        super().__init__("dynamic_tf_turtlesim")  # node name
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.on_timer)
        self.spawner = self.create_client(Spawn, "spawn")
        request = Spawn.Request()
        request.x = 3.0
        request.y = 3.0
        request.theta = 0.0
        self.result = self.spawner.call_async(request)
        print("done")
        self.tf_br = TransformBroadcaster(self)
        self.sub = self.create_subscription(Pose, "/turtle1/pose", self.sub_cb, 10)
        self.sub2 = self.create_subscription(Pose, "/turtle2/pose", self.sub_cb2, 10)
        self.pub = self.create_publisher(Twist, "/turtle2/cmd_vel", 10)

    def sub_cb(self, msg: Pose):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "turtle1"
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0
        quat = quaternion_from_euler(0, 0, msg.theta)  # r y p
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.tf_br.sendTransform(t)

    def sub_cb2(self, msg: Pose):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "turtle2"
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0
        quat = quaternion_from_euler(0, 0, msg.theta)  # r y p
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.tf_br.sendTransform(t)

    def on_timer(self):
        try:
            t = self.tf_buffer.lookup_transform("turtle1", "turtle2", time.Time())
        except Exception:
            self.get_logger().info("lookup 실패!!")
            return
        self.get_logger().info(f"{t.transform.translation.x}")
        self.get_logger().info(f"{t.transform.translation.y}")
        self.get_logger().info(f"{t.transform.translation.z}")
        msg = Twist()
        angular = euler_from_quaternion(
            (
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w,
            )
        )
        msg.angular.x = angular[0]
        msg.angular.y = angular[1]
        msg.angular.z = angular[2]
        msg.linear.x = t.transform.translation.x + t.transform.translation.y
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = dynamic_tf_turtlesim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()


if __name__ == "__main__":
    main()