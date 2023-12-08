import unittest
from unittest.mock import Mock

import rclpy
import rclpy.node
import std_msgs.msg

import vscode_py.incrementer_node


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

    def test_increments_number_10Plus(self):
        # Call callback manually
        self.incrementer.callback(std_msgs.msg.Int32(data=10))

        # Assert that the published message was incremented
        self.assertEqual(self.incrementer.publisher.publish.call_args[0][0].data, 13)

        # Call callback manually
        self.incrementer.callback(std_msgs.msg.Int32(data=11))

        # Assert that the published message was incremented
        self.assertEqual(self.incrementer.publisher.publish.call_args[0][0].data, 21)

    def test_increments_number_even(self):
        # Call callback manually
        self.incrementer.callback(std_msgs.msg.Int32(data=4))

        # Assert that the published message was incremented
        self.assertEqual(self.incrementer.publisher.publish.call_args[0][0].data, 7)


if __name__ == "__main__":
    suite = unittest.TestSuite()

    # Add all tests in this file to the suite
    suite.addTest(TestIncrementer("test_publishes_in_callback"))
    suite.addTest(TestIncrementer("test_increments_number"))
    suite.addTest(TestIncrementer("test_increments_number_10Plus"))
    suite.addTest(TestIncrementer("test_increments_number_even"))

    runner = unittest.TextTestRunner()
    runner.run(suite)
