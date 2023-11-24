import rclpy
import rclpy.node

from rclpy.executors import ExternalShutdownException

import std_msgs.msg


class Incrementer:
    def __init__(self, node: rclpy.node.Node):
        print("Init node")

        self.node = node

        # Create publisher
        self.publisher = self.node.create_publisher(
            msg_type=std_msgs.msg.Int32, topic="incremented_number", qos_profile=10
        )

        # Create subscriber
        self.subscriber = self.node.create_subscription(
            msg_type=std_msgs.msg.Int32,
            topic="number",
            callback=self.callback,
            qos_profile=10,
        )

    def callback(self, msg: std_msgs.msg.Int32):
        print(f"Callback called with number: {msg.data}")

        number = msg.data
        incremented_number = number + 1

        # Publish incremented number
        self.publisher.publish(std_msgs.msg.Int32(data=incremented_number))


def main(args=None):
    # Init rclpy
    rclpy.init(args=args)

    # Create node
    node = rclpy.create_node("incrementer")
    _ = Incrementer(node)

    # Spin node until destroyed
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
