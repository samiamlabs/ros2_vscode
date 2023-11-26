import os
import signal
import subprocess
import sys
import time

import junit_xml
import rclpy
import std_msgs.msg


class ROS2LaunchManager:
    def __init__(self):
        self.process = None

    def __enter__(self):
        # Start the ROS2 launch file as a subprocess
        launchfile = "incr_relay_system.launch.py"
        # launchfile = "cpp_incr_relay_system.launch.py"

        self.process = subprocess.Popen(
            ["ros2", "launch", "vscode_bringup", launchfile],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid,
        )

        return self.process

    def __exit__(self, exc_type, exc_value, traceback):
        self.terminate_ros_launch()

    def terminate_ros_launch(self):
        if self.process:
            # Send the SIGTERM signal to the process group id (pgid)
            os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
            # Wait for the process to terminate
            self.process.wait()
            print("ROS2 launch process terminated")


class TestRunner:
    def __init__(self):
        rclpy.init()

        self.test_node = rclpy.create_node("test_node")
        self.number_pub = self.test_node.create_publisher(
            std_msgs.msg.Int32, "number", 10
        )
        self.number_sub = self.test_node.create_subscription(
            std_msgs.msg.Int32, "number", self.number_callback, 10
        )

        self.received_numbers = []

    def number_callback(self, msg: std_msgs.msg.Int32):
        print(f"Got number: {msg.data}")
        self.received_numbers.append(msg.data)

    def run_tests(self):
        print("Running tests...")

        test_cases = []
        test_result = True
        test_case = junit_xml.TestCase(
            "Counts up number. 1 2 3 etc", "System Behavior Tests"
        )

        print("Waiting for nodes to start up...")
        time.sleep(5)

        test_start = time.time()

        print("Publishing first number...")
        self.number_pub.publish(std_msgs.msg.Int32(data=1))

        print("Waiting for five numbers to be received or for timeout...")
        start_time = time.time()
        while len(self.received_numbers) < 5 and time.time() < start_time + 11:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)
            time.sleep(0.1)

        print("Checking results...")

        test_case.elapsed_sec = time.time() - test_start

        try:
            assert len(self.received_numbers) >= 5
            assert (
                self.received_numbers[0] == 1
            ), f"Expected {self.received_numbers[0]} to be 1"
            assert (
                self.received_numbers[1] == 2
            ), f"Expected {self.received_numbers[1]} to be 2"
            assert (
                self.received_numbers[2] == 3
            ), f"Expected {self.received_numbers[2]} to be 3"
            assert (
                self.received_numbers[3] == 4
            ), f"Expected {self.received_numbers[3]} to be 4"
            assert (
                self.received_numbers[4] == 5
            ), f"Expected {self.received_numbers[4]} to be 5"
        except AssertionError as error:
            print(f"Assertion error: {error}")
            test_result = False
            test_case.add_failure_info(message=error)

        if test_result:
            print("Tests passed!")
        else:
            print("Tests failed!")

        test_cases.append(test_case)

        return test_cases


def signal_handler(signum, frame):
    print("Signal handler called with signal:", signum)
    sys.exit(0)


# Set up signal handlers to make sure termination is handled on signals
# NOTE(sam): Does not seem to help
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)


# Write test results to a JUnit XML file
def write_junit_xml(test_cases, xml_file_path):
    test_suite = junit_xml.TestSuite("IncrementorRelaySystem", test_cases)
    with open(xml_file_path, "w") as f:
        junit_xml.TestSuite.to_file(f, [test_suite], prettyprint=True)


def generate_html_report(xml_file_path, html_file_path):
    subprocess.run(
        ["python3", "-m" "junit2htmlreport", xml_file_path, html_file_path],
        check=True,
    )


def main():
    with ROS2LaunchManager():
        # Insert testing logic here
        test_runner = TestRunner()
        test_cases = test_runner.run_tests()

        # Write test results to file
        junit_xml_path = (
            "/home/ros/ws/src/vscode_py/test_system/results/test_results.xml"
        )
        write_junit_xml(test_cases, junit_xml_path)

        # Generate a HTML report from the JUnit XML file
        output_html_path = (
            "/home/ros/ws/src/vscode_py/test_system/results/test_results.html"
        )
        generate_html_report(junit_xml_path, output_html_path)

        print("Tests are done running! Exiting...")


if __name__ == "__main__":
    main()
