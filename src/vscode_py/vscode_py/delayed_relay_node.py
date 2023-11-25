import rclpy
import rclpy.node

import std_msgs.msg

from rclpy.executors import ExternalShutdownException
from rclpy.callback_groups import ReentrantCallbackGroup
from vscode_py.async_primitives import async_sleep


class DelayedRelay:
    def __init__(self, node: rclpy.node.Node):
        self.node = node
        self.node.get_logger().info("Initializing delayed relay node")

        # Change the default callback group to allow for async callbacks
        self.node._default_callback_group = ReentrantCallbackGroup()

        # Create subscriber
        self.number_sub = node.create_subscription(
            std_msgs.msg.Int32, "number", self.callback, 10
        )

        # Create publisher
        self.number_pub = node.create_publisher(
            std_msgs.msg.Int32, "delayed_number", 10
        )

    async def callback(self, msg: std_msgs.msg.Int32):
        # Sleep for one second
        await async_sleep(self.node, 1.0)

        # Publish message
        self.number_pub.publish(std_msgs.msg.Int32(data=msg.data))


def main(args=None):
    # Init rclpy
    rclpy.init(args=args)

    # Create node
    node = rclpy.create_node("delayed_relay")
    _ = DelayedRelay(node)

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
