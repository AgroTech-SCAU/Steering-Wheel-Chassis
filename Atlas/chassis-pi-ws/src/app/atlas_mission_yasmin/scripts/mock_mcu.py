#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class MockMcu(Node):
    def __init__(self):
        super().__init__("atlas_mock_mcu")
        self.get_logger().info("mock MCU skeleton started; behavior is added in Step 7")


def main():
    rclpy.init()
    node = MockMcu()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
