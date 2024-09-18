import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SubNode(Node):
    def __init__(self):
        super().__init__("sub_node")
        self.create_subscription(String, "my_topic", self.sub_callback, 10)

    def sub_callback(self, msg: String):
        self.get_logger().info("Data from topic: {}".format(msg.data))


def main():
    rclpy.init()
    node = SubNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
