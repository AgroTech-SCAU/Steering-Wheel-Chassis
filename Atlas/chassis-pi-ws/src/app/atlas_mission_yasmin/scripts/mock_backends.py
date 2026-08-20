#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class MockBackends(Node):
    def __init__(self):
        super().__init__("atlas_mock_backends")
        self.get_logger().info("mock backend skeleton started; behavior is added in Step 7")


def main():
    rclpy.init()
    node = MockBackends()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
