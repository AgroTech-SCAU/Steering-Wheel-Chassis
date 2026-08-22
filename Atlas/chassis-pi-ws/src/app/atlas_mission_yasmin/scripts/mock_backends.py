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

"""Mock navigation and manipulation backends for Atlas YASMIN tests."""

import json

import rclpy
from atlas_mission_interfaces.msg import ManipulationStatus
from atlas_mission_interfaces.msg import NavigationStatus
from atlas_mission_interfaces.srv import CancelManipulation
from atlas_mission_interfaces.srv import CancelNavigation
from atlas_mission_interfaces.srv import StartManipulation
from atlas_mission_interfaces.srv import StartNavigation
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import String


class MockBackends(Node):
    def __init__(self):
        super().__init__("atlas_mock_backends")
        self.declare_parameter("scenario", "normal")
        self.declare_parameter("navigation_delay_s", 0.18)
        self.declare_parameter("interrupt_navigation_delay_s", 2.0)
        self.declare_parameter("manipulation_delay_s", 0.12)
        self.scenario = self.get_parameter("scenario").get_parameter_value().string_value

        self.active_navigation = None
        self.active_manipulation = None

        trace_qos = QoSProfile(depth=100)
        trace_qos.reliability = ReliabilityPolicy.RELIABLE
        trace_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.trace_pub = self.create_publisher(
            String, "/atlas/mission_yasmin_mock/trace", trace_qos)
        self.nav_status_pub = self.create_publisher(
            NavigationStatus, "/atlas/navigation/status", 10)
        self.nav_cmd_pub = self.create_publisher(Twist, "/atlas/navigation/cmd_vel", 10)
        self.manip_status_pub = self.create_publisher(
            ManipulationStatus, "/atlas/manipulation/status", 10)

        self.nav_start_srv = self.create_service(
            StartNavigation, "/atlas/navigation/start", self._on_nav_start)
        self.nav_cancel_srv = self.create_service(
            CancelNavigation, "/atlas/navigation/cancel", self._on_nav_cancel)
        self.manip_start_srv = self.create_service(
            StartManipulation, "/atlas/manipulation/start", self._on_manip_start)
        self.manip_cancel_srv = self.create_service(
            CancelManipulation, "/atlas/manipulation/cancel", self._on_manip_cancel)
        self.cmd_timer = self.create_timer(0.05, self._publish_cmd_vel)
        self.get_logger().info(f"mock backends started with scenario={self.scenario}")

    def _make_timer(self, delay_s, callback):
        timer = None

        def _wrapped():
            timer.cancel()
            callback()

        timer = self.create_timer(delay_s, _wrapped)
        return timer

    def _trace(self, **payload):
        payload.setdefault("source", "mock_backends")
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.trace_pub.publish(msg)

    def _publish_nav_status(self, state, request, message, stamp=None):
        status = NavigationStatus()
        status.header.stamp = stamp if stamp is not None else self.get_clock().now().to_msg()
        status.state = state
        status.backend = request.backend
        status.waypoint_id = request.waypoint_id
        status.target_x_m = request.x_m
        status.target_y_m = request.y_m
        status.target_yaw_rad = request.yaw_rad
        status.message = message
        self.nav_status_pub.publish(status)

    def _publish_manip_status(self, state, request, message, stamp=None):
        status = ManipulationStatus()
        status.header.stamp = stamp if stamp is not None else self.get_clock().now().to_msg()
        status.state = state
        status.backend = request.backend
        status.waypoint_id = request.waypoint_id
        status.task_id = request.arrival_task
        status.step_name = request.prepare_action
        status.message = message
        self.manip_status_pub.publish(status)

    def _on_nav_start(self, request, response):
        self._trace(
            event="nav_start",
            waypoint_id=request.waypoint_id,
            reset_origin=request.reset_origin,
        )
        if self.scenario == "navigation_rejected":
            response.success = False
            response.message = "mock navigation rejected"
            return response

        response.success = True
        response.message = "mock navigation accepted"
        self.active_navigation = request
        self._publish_nav_status(
            NavigationStatus.STATE_RUNNING, request, "mock navigation running")
        if self.scenario == "stale_navigation_status" and request.waypoint_id == "row_1_entry":
            stale_stamp = (self.get_clock().now() - Duration(seconds=30)).to_msg()
            self._publish_nav_status(
                NavigationStatus.STATE_SUCCEEDED,
                request,
                "mock stale navigation success",
                stamp=stale_stamp,
            )
            self._trace(event="nav_stale_terminal", waypoint_id=request.waypoint_id)

        if self.scenario in (
            "reset_during_navigation",
            "fault_during_navigation",
            "estop_during_navigation",
            "mcu_timeout_during_navigation",
        ):
            delay = self.get_parameter("interrupt_navigation_delay_s").value
        else:
            delay = self.get_parameter("navigation_delay_s").value
        self._make_timer(delay, lambda: self._finish_navigation(request))
        return response

    def _finish_navigation(self, request):
        if self.active_navigation is not request:
            return
        if self.scenario == "navigation_failed" and request.waypoint_id == "row_1_entry":
            state = NavigationStatus.STATE_FAILED
            event = "nav_failed"
            message = "mock navigation failed"
        elif self.scenario == "navigation_timeout":
            return
        else:
            state = NavigationStatus.STATE_SUCCEEDED
            event = "nav_succeeded"
            message = "mock navigation succeeded"
        self.active_navigation = None
        self._publish_nav_status(state, request, message)
        self._trace(event=event, waypoint_id=request.waypoint_id)

    def _on_nav_cancel(self, request, response):
        response.success = True
        response.message = "mock navigation cancelled"
        if self.active_navigation is not None:
            active = self.active_navigation
            self.active_navigation = None
            self._publish_nav_status(
                NavigationStatus.STATE_CANCELLED, active, "mock navigation cancelled")
        self._trace(event="nav_cancel", reason=request.reason)
        return response

    def _on_manip_start(self, request, response):
        self._trace(
            event="manip_start",
            waypoint_id=request.waypoint_id,
            task_id=request.arrival_task,
            prepare_action=request.prepare_action,
        )
        if self.scenario == "manipulation_rejected":
            response.success = False
            response.message = "mock manipulation rejected"
            return response

        response.success = True
        response.message = "mock manipulation accepted"
        self.active_manipulation = request
        self._publish_manip_status(
            ManipulationStatus.STATE_RUNNING, request, "mock manipulation running")
        if self.scenario != "manipulation_timeout":
            self._make_timer(
                self.get_parameter("manipulation_delay_s").value,
                lambda: self._finish_manipulation(request),
            )
        return response

    def _finish_manipulation(self, request):
        if self.active_manipulation is not request:
            return
        if self.scenario == "manipulation_failed":
            state = ManipulationStatus.STATE_FAILED
            event = "manip_failed"
            message = "mock manipulation failed"
        else:
            state = ManipulationStatus.STATE_SUCCEEDED
            event = "manip_succeeded"
            message = "mock manipulation succeeded"
        self.active_manipulation = None
        self._publish_manip_status(state, request, message)
        self._trace(event=event, waypoint_id=request.waypoint_id, task_id=request.arrival_task)

    def _on_manip_cancel(self, request, response):
        response.success = True
        response.message = "mock manipulation cancelled"
        if self.active_manipulation is not None:
            active = self.active_manipulation
            self.active_manipulation = None
            self._publish_manip_status(
                ManipulationStatus.STATE_CANCELLED, active, "mock manipulation cancelled")
        self._trace(event="manip_cancel", reason=request.reason)
        return response

    def _publish_cmd_vel(self):
        if self.active_navigation is None:
            return
        cmd = Twist()
        cmd.linear.x = 0.15
        self.nav_cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = MockBackends()
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
