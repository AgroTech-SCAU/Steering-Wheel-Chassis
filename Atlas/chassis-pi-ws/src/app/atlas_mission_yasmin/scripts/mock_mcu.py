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

"""Mock MCU for Atlas YASMIN launch acceptance tests."""

import json

import rclpy
from mcu_comm_bridge.msg import AutoTaskEvent
from mcu_comm_bridge.msg import McuStatus
from mcu_comm_bridge.srv import ReportMissionResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool


class MockMcu(Node):
    def __init__(self):
        super().__init__("atlas_mock_mcu")
        self.declare_parameter("scenario", "normal")
        self.declare_parameter("status_period_s", 0.05)
        self.declare_parameter("start_delay_s", 0.8)
        self.declare_parameter("interrupt_delay_s", 0.35)

        self.scenario = self.get_parameter("scenario").get_parameter_value().string_value
        self.app_state = McuStatus.STATE_AUTO_PI
        self.ready_flags = 0xFF
        self.online_flags = 0xFF
        self.auto_start_latched = False
        self.publish_status = True
        self._interrupt_scheduled = False
        self._sent_count = 0

        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        trace_qos = QoSProfile(depth=100)
        trace_qos.reliability = ReliabilityPolicy.RELIABLE
        trace_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_pub = self.create_publisher(McuStatus, "/mcu/status", status_qos)
        self.event_pub = self.create_publisher(AutoTaskEvent, "/mcu/auto_task_event", 10)
        self.trace_pub = self.create_publisher(
            String, "/atlas/mission_yasmin_mock/trace", trace_qos)
        self.trace_sub = self.create_subscription(
            String, "/atlas/mission_yasmin_mock/trace", self._on_trace, trace_qos)
        self.report_srv = self.create_service(
            ReportMissionResult, "/mcu/report_mission_result", self._on_report)
        self.brake_srv = self.create_service(SetBool, "/mcu/set_brake", self._on_brake)

        status_period = self.get_parameter("status_period_s").value
        self.status_timer = self.create_timer(status_period, self._publish_status)
        self._make_timer(self.get_parameter("start_delay_s").value, self._publish_start)
        self.get_logger().info(f"mock MCU started with scenario={self.scenario}")

    def _make_timer(self, delay_s, callback):
        timer = None

        def _wrapped():
            timer.cancel()
            callback()

        timer = self.create_timer(delay_s, _wrapped)
        return timer

    def _trace(self, **payload):
        payload.setdefault("source", "mock_mcu")
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.trace_pub.publish(msg)

    def _publish_status(self):
        if not self.publish_status:
            return
        status = McuStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.app_state = self.app_state
        status.ready_flags = self.ready_flags
        status.online_flags = self.online_flags
        status.auto_start_latched = self.auto_start_latched
        self.status_pub.publish(status)

    def _publish_start(self):
        self.auto_start_latched = True
        event = AutoTaskEvent()
        event.header.stamp = self.get_clock().now().to_msg()
        event.event = AutoTaskEvent.EVENT_START
        event.app_state = self.app_state
        event.auto_start_latched = True
        self.event_pub.publish(event)
        self._trace(event="mcu_start")

    def _publish_reset(self):
        event = AutoTaskEvent()
        event.header.stamp = self.get_clock().now().to_msg()
        event.event = AutoTaskEvent.EVENT_RESET
        event.app_state = self.app_state
        event.auto_start_latched = False
        self.event_pub.publish(event)
        self._trace(event="mcu_reset")

    def _on_trace(self, msg):
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if event.get("event") != "nav_start" or self._interrupt_scheduled:
            return
        if event.get("waypoint_id") != "row_1_entry":
            return
        handlers = {
            "reset_during_navigation": self._publish_reset,
            "fault_during_navigation": self._enter_fault,
            "estop_during_navigation": self._enter_estop,
            "mcu_timeout_during_navigation": self._stop_status,
        }
        handler = handlers.get(self.scenario)
        if handler is None:
            return
        self._interrupt_scheduled = True
        self._make_timer(self.get_parameter("interrupt_delay_s").value, handler)

    def _enter_fault(self):
        self.app_state = McuStatus.STATE_FAULT
        self._publish_status()
        self._trace(event="mcu_fault")

    def _enter_estop(self):
        self.app_state = McuStatus.STATE_ESTOP
        self._publish_status()
        self._trace(event="mcu_estop")

    def _stop_status(self):
        self.publish_status = False
        self._trace(event="mcu_timeout_started")

    def _on_report(self, request, response):
        self._sent_count += 1
        response.success = True
        response.message = "mock accepted mission result"
        response.sent_count = self._sent_count
        result = "DONE" if request.result == ReportMissionResult.Request.RESULT_DONE else "FAIL"
        self._trace(event="report_result", result=result, code=request.code)
        return response

    def _on_brake(self, request, response):
        response.success = True
        response.message = "mock brake set"
        self._trace(event="set_brake", enabled=request.data)
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
