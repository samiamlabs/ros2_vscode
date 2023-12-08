import rclpy
import rclpy.node
import std_msgs.msg
from rclpy.executors import ExternalShutdownException


class IncrementerController:
    def __init__(self, node: rclpy.node.Node):
        self.node = node

        self.node.get_logger().info("Initailizing incrementer_controller node")

        # Create publisher
        self.publisher = self.node.create_publisher(
            msg_type=std_msgs.msg.Int32, topic="direction", qos_profile=10
        )

        # Create subscriber
        self.subscriber = self.node.create_subscription(
            msg_type=std_msgs.msg.Int32,
            topic="incremented_number",
            callback=self.incremented_number_callback,
            qos_profile=10,
        )

    def incremented_number_callback(self, msg: std_msgs.msg.Int32):
        self.node.get_logger().info(f"In callback with number: {msg.data}")

        number = msg.data
        if number >= 50:
            direction = 1
        else:
            direction = 0

        # Publish direction
        self.publisher.publish(std_msgs.msg.Int32(data=direction))
