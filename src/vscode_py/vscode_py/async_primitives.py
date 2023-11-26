import rclpy
import rclpy.task
from rclpy.node import Node


async def async_sleep(node: Node, seconds: float, callback_group=None):
    future = rclpy.task.Future()

    def handler():
        future.set_result(None)

    timer = None
    if callback_group is None:
        timer = node.create_timer(seconds, handler)
    else:
        timer = node.create_timer(seconds, handler, callback_group=callback_group)
    await future

    timer.cancel()
    timer.destroy()

    return None


async def future_with_timeout(
    node: Node, future: rclpy.task.Future, timeout_sec: float
):
    first_done_future = rclpy.task.Future()

    def handler(arg=None):
        first_done_future.set_result(None)

    timer = node.create_timer(timeout_sec, handler)
    future.add_done_callback(handler)

    await first_done_future

    if not future.done():
        raise TimeoutError(f"Future timed out after {timeout_sec} seconds")

    timer.cancel()
    timer.destroy()

    return future.result()
