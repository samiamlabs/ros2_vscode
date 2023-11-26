import unittest
import time

import vscode_py.delayed_relay_node

import rclpy
import std_msgs.msg


class TestDeleyedRelay(unittest.TestCase):
    def setUp(self):
        # Settings
        self.default_timeout = 3

        rclpy.init(args=None)
        self.node = rclpy.create_node("delayed_relay")
        self.delayd_relay = vscode_py.delayed_relay_node.DelayedRelay(self.node)

        self.test_node = rclpy.create_node("test_node")
        self.test_publisher = self.test_node.create_publisher(
            std_msgs.msg.Int32, "number", 10
        )

        self.delayed_msg = None
        self.delayed_msg_count = 0

        self.test_subscription = self.test_node.create_subscription(
            std_msgs.msg.Int32, "delayed_number", self.delayed_msg_callback, 10
        )

    def delayed_msg_callback(self, msg: std_msgs.msg.Int32):
        self.delayed_msg = msg
        self.delayed_msg_count += 1

    def tearDown(self):
        self.node.destroy_node()
        self.test_node.destroy_node()
        rclpy.shutdown()

    def test_forwards_message(self):
        # Publish message to delayed_relay
        self.test_publisher.publish(std_msgs.msg.Int32(data=1))

        # Spin node until it has published or the default timeout has passed
        start_time = time.time()
        while (
            self.delayed_msg is None and time.time() < start_time + self.default_timeout
        ):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            rclpy.spin_once(self.test_node, timeout_sec=0.1)

        # Assert that the message was forwarded
        self.assertEqual(self.delayed_msg.data, 1)

    def test_delays_message_one_second(self):
        # Publish message to delayed_relay
        self.test_publisher.publish(std_msgs.msg.Int32(data=1))

        # Spin node until it has published or the default timeout has passed
        start_time = time.time()
        while (
            self.delayed_msg is None and time.time() < start_time + self.default_timeout
        ):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            rclpy.spin_once(self.test_node, timeout_sec=0.1)

        # Assert that it took about one second to forward the message
        self.assertAlmostEqual(time.time() - start_time, 1, delta=0.1)

    def test_handles_parallel_messages(self):
        # Publish message to delayed_relay
        self.test_publisher.publish(std_msgs.msg.Int32(data=1))
        self.test_publisher.publish(std_msgs.msg.Int32(data=2))

        # Spin node until it has published or the default timeout has passed
        start_time = time.time()
        while (
            self.delayed_msg_count < 2
            and time.time() < start_time + self.default_timeout
        ):
            rclpy.spin_once(self.node, timeout_sec=0.01)
            rclpy.spin_once(self.test_node, timeout_sec=0.01)

        # Assert that the message was forwarded
        self.assertEqual(self.delayed_msg_count, 2)

        # Assert that it took about one second to forward the messages
        self.assertAlmostEqual(time.time() - start_time, 1, delta=0.1)


if __name__ == "__main__":
    suite = unittest.TestSuite()

    # Add all tests in this file to the suite
    suite.addTest(TestDeleyedRelay("test_forwards_message"))
    suite.addTest(TestDeleyedRelay("test_delays_message_one_second"))

    runner = unittest.TextTestRunner()
    runner.run(suite)
