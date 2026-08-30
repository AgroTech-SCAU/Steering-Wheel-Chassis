#!/usr/bin/env python3
# Copyright 2026 yangxuan
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Minimal MCU mock: publish AUTO_PI status, then one latched AUTO event."""

import rclpy
from mcu_comm_bridge.msg import AutoTaskEvent, McuStatus
from mcu_comm_bridge.srv import ReportMissionResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import SetBool


class MockMcu(Node):
    def __init__(self):
        super().__init__("atlas_mock_mcu")
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_pub = self.create_publisher(McuStatus, "/mcu/status", qos)
        self.event_pub = self.create_publisher(AutoTaskEvent, "/mcu/auto_task_event", 10)
        self.create_service(
            ReportMissionResult, "/mcu/report_mission_result", self._report)
        self.create_service(SetBool, "/mcu/set_brake", self._brake)
        self.auto_latched = False
        self.create_timer(0.05, self._status)
        self.start_timer = self.create_timer(0.8, self._start_once)

    def _status(self):
        msg = McuStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.app_state = McuStatus.STATE_AUTO_PI
        msg.ready_flags = 0xFF
        msg.online_flags = 0xFF
        msg.auto_start_latched = self.auto_latched
        self.status_pub.publish(msg)

    def _start_once(self):
        self.start_timer.cancel()
        self.auto_latched = True
        msg = AutoTaskEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.event = AutoTaskEvent.EVENT_START
        msg.app_state = McuStatus.STATE_AUTO_PI
        msg.auto_start_latched = True
        self.event_pub.publish(msg)

    def _report(self, request, response):
        del request
        response.success = True
        response.message = "mock result accepted"
        response.sent_count = 1
        return response

    def _brake(self, request, response):
        del request
        response.success = True
        response.message = "mock brake accepted"
        return response


def main():
    rclpy.init()
    node = MockMcu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
