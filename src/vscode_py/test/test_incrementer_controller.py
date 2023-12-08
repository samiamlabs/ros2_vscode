import unittest
from unittest.mock import Mock

import rclpy
import rclpy.node
import std_msgs.msg

import vscode_py.incrementer_controller_node


class TestIncrementerController(unittest.TestCase):
    def setUp(self):
        # Create mock node
        self.node = Mock(rclpy.node.Node)
        # Create incrementer with mock node
        self.incrementer_controller = (
            vscode_py.incrementer_controller_node.IncrementerController(self.node)
        )

    def test_threshold50(self):
        # Call callback manually
        self.incrementer_controller.incremented_number_callback(
            std_msgs.msg.Int32(data=49)
        )

        # Assert that the incrementer published to the mock node
        self.assertEqual(self.incrementer_controller.publisher.publish.call_count, 1)
        self.assertEqual(
            self.incrementer_controller.publisher.publish.call_args[0][0].data, 0
        )

        # Call callback manually
        self.incrementer_controller.incremented_number_callback(
            std_msgs.msg.Int32(data=51)
        )

        # Assert that the incrementer published to the mock node
        self.assertEqual(self.incrementer_controller.publisher.publish.call_count, 2)
        self.assertEqual(
            self.incrementer_controller.publisher.publish.call_args[0][0].data, 1
        )


if __name__ == "__main__":
    suite = unittest.TestSuite()

    # Add all tests in this file to the suite
    suite.addTest(TestIncrementerController("test_threshold50"))
    runner = unittest.TextTestRunner()
    unittest.TextTestRunner().run(suite)
