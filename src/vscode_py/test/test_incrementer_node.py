import unittest
from unittest.mock import Mock

import rclpy
import rclpy.node

import vscode_py.incrementer_node

import std_msgs.msg


class TestIncrementer(unittest.TestCase):
    def setUp(self):
        # Create mock node
        self.node = Mock(rclpy.node.Node)
        # Create incrementer with mock node
        self.incrementer = vscode_py.incrementer_node.Incrementer(self.node)

    def test_publishes_in_callback(self):
        # Call callback manually
        self.incrementer.callback(std_msgs.msg.Int32(data=1))

        # Assert that the incrementer published to the mock node
        self.assertEqual(self.incrementer.publisher.publish.call_count, 1)

    def test_increments_number(self):
        # Call callback manually
        self.incrementer.callback(std_msgs.msg.Int32(data=1))

        # Assert that the published message was incremented
        self.assertEqual(self.incrementer.publisher.publish.call_args[0][0].data, 2)


if __name__ == "__main__":
    suite = unittest.TestSuite()

    # Add all tests in this file to the suite
    suite.addTest(TestIncrementer("test_publishes_in_callback"))
    suite.addTest(TestIncrementer("test_increments_number"))

    runner = unittest.TextTestRunner()
    runner.run(suite)
