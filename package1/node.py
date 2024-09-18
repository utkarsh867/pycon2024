import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MyNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.publisher = self.create_publisher(String, "my_topic", 10)

        self.create_timer(
            1.0,
            self.timer_callback,
        )

    def timer_callback(self):
        self.get_logger().info("Publishing Hello, World! to topic my_topic")
        msg = String(data="Hello, World!")
        self.publisher.publish(msg)


def main():
    rclpy.init()

    node = MyNode("my_node")
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
