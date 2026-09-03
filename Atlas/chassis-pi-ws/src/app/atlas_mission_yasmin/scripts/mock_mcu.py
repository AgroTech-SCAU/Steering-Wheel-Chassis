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

"""Deterministic MCU mock for competition mission acceptance scenarios."""

import rclpy
from atlas_mission_interfaces.msg import NavigationStatus
from mcu_comm_bridge.msg import AutoTaskEvent, McuStatus
from mcu_comm_bridge.srv import ReportMissionResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import SetBool


TIMED_INTERRUPT_SCENARIOS = {
    "reset_during_navigation",
    "fault_during_navigation",
    "estop_during_navigation",
}


class MockMcu(Node):
    def __init__(self):
        super().__init__("atlas_mock_mcu")
        self.declare_parameter("scenario", "normal")
        self.declare_parameter("auto_start_delay_s", 0.8)
        self.declare_parameter("interrupt_delay_s", 0.35)

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_pub = self.create_publisher(McuStatus, "/mcu/status", qos)
        self.event_pub = self.create_publisher(
            AutoTaskEvent, "/mcu/auto_task_event", 10)
        self.create_service(
            ReportMissionResult, "/mcu/report_mission_result", self._report)
        self.create_service(SetBool, "/mcu/set_brake", self._brake)
        self.create_subscription(
            NavigationStatus,
            "/atlas/navigation/status",
            self._on_navigation_status,
            10,
        )

        self.auto_latched = False
        self.app_state = McuStatus.STATE_AUTO_PI
        self.status_enabled = True
        self.mcu_timeout_armed = False

        self.create_timer(0.05, self._status)
        self.start_timer = self.create_timer(
            self.get_parameter("auto_start_delay_s").value, self._start_once)
        self.scenario_timer = None

    def _once(self, delay_s, callback):
        timer = None

        def wrapped():
            timer.cancel()
            callback()

        timer = self.create_timer(delay_s, wrapped)
        return timer

    def _status(self):
        if not self.status_enabled:
            return
        msg = McuStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.app_state = self.app_state
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

        scenario = self.get_parameter("scenario").value
        if scenario in TIMED_INTERRUPT_SCENARIOS:
            self.scenario_timer = self._once(
                self.get_parameter("interrupt_delay_s").value,
                self._trigger_scenario,
            )

    def _on_navigation_status(self, msg):
        if (
            self.get_parameter("scenario").value
            != "mcu_timeout_during_navigation"
        ):
            return
        if self.mcu_timeout_armed:
            return
        if (
            msg.state == NavigationStatus.STATE_RUNNING
            and msg.waypoint_id == "pickup"
        ):
            self.mcu_timeout_armed = True
            self.scenario_timer = self._once(
                self.get_parameter("interrupt_delay_s").value,
                self._trigger_scenario,
            )

    def _trigger_scenario(self):
        scenario = self.get_parameter("scenario").value
        if scenario == "reset_during_navigation":
            self.auto_latched = False
            msg = AutoTaskEvent()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.event = AutoTaskEvent.EVENT_RESET
            msg.app_state = McuStatus.STATE_AUTO_PI
            msg.auto_start_latched = False
            self.event_pub.publish(msg)
        elif scenario == "fault_during_navigation":
            self.app_state = McuStatus.STATE_FAULT
            self._status()
        elif scenario == "estop_during_navigation":
            self.app_state = McuStatus.STATE_ESTOP
            self._status()
        elif scenario == "mcu_timeout_during_navigation":
            self.status_enabled = False

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
