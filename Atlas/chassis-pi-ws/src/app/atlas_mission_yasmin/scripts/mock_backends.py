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

"""Minimal mock navigation, vision and arm backends for the competition YASMIN flow."""

import rclpy
from atlas_mission_interfaces.msg import ManipulationStatus, NavigationStatus
from atlas_mission_interfaces.srv import (
    CancelManipulation,
    CancelNavigation,
    ClassifySortingRule,
    DetectCameraTarget,
    StartManipulation,
    StartNavigation,
)
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_srvs.srv import SetBool


class MockBackends(Node):
    def __init__(self):
        super().__init__("atlas_mock_backends")
        self.declare_parameter("arena", "A")
        self.declare_parameter("navigation_delay_s", 0.10)
        self.declare_parameter("manipulation_delay_s", 0.08)

        self.active_navigation = None
        self.active_manipulation = None

        self.nav_status_pub = self.create_publisher(
            NavigationStatus, "/atlas/navigation/status", 10)
        self.nav_cmd_pub = self.create_publisher(Twist, "/atlas/navigation/cmd_vel", 10)
        self.manip_status_pub = self.create_publisher(
            ManipulationStatus, "/atlas/manipulation/status", 10)

        self.create_service(StartNavigation, "/atlas/navigation/start", self._nav_start)
        self.create_service(CancelNavigation, "/atlas/navigation/cancel", self._nav_cancel)
        self.create_service(SetBool, "/atlas/navigation/view_scan", self._view_scan)
        self.create_service(StartManipulation, "/atlas/manipulation/start", self._manip_start)
        self.create_service(CancelManipulation, "/atlas/manipulation/cancel", self._manip_cancel)
        self.create_service(
            ClassifySortingRule,
            "/atlas/vision/classify_sorting_rule",
            self._classify_sorting,
        )
        self.create_service(DetectCameraTarget, "/atlas/vision/detect_target", self._detect)

        self.create_timer(0.05, self._publish_cmd_vel)

    def _once(self, delay_s, callback):
        timer = None

        def wrapped():
            timer.cancel()
            callback()

        timer = self.create_timer(delay_s, wrapped)

    def _nav_status(self, request, state, message):
        msg = NavigationStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = state
        msg.backend = request.backend
        msg.waypoint_id = request.waypoint_id
        msg.message = message
        self.nav_status_pub.publish(msg)

    def _manip_status(self, request, state, message):
        msg = ManipulationStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = state
        msg.backend = request.backend
        msg.waypoint_id = request.waypoint_id
        msg.task_id = request.arrival_task
        msg.step_name = request.prepare_action
        msg.message = message
        self.manip_status_pub.publish(msg)

    def _nav_start(self, request, response):
        if request.arena not in ("A", "B"):
            response.success = False
            response.message = "arena must be A or B"
            return response
        response.success = True
        response.message = "accepted"
        self.active_navigation = request
        self._nav_status(request, NavigationStatus.STATE_RUNNING, "running")
        self._once(self.get_parameter("navigation_delay_s").value,
                   lambda: self._finish_nav(request))
        return response

    def _finish_nav(self, request):
        if self.active_navigation is not request:
            return
        self.active_navigation = None
        self._nav_status(request, NavigationStatus.STATE_SUCCEEDED, "succeeded")

    def _nav_cancel(self, request, response):
        response.success = True
        response.message = request.reason
        if self.active_navigation is not None:
            active = self.active_navigation
            self.active_navigation = None
            self._nav_status(active, NavigationStatus.STATE_CANCELLED, "cancelled")
        return response

    def _view_scan(self, request, response):
        response.success = True
        response.message = "scan pose" if request.data else "standard pose"
        return response

    def _manip_start(self, request, response):
        response.success = True
        response.message = "accepted"
        self.active_manipulation = request
        self._manip_status(request, ManipulationStatus.STATE_RUNNING, "running")
        self._once(self.get_parameter("manipulation_delay_s").value,
                   lambda: self._finish_manip(request))
        return response

    def _finish_manip(self, request):
        if self.active_manipulation is not request:
            return
        self.active_manipulation = None
        self._manip_status(request, ManipulationStatus.STATE_SUCCEEDED, "succeeded")

    def _manip_cancel(self, request, response):
        response.success = True
        response.message = request.reason
        if self.active_manipulation is not None:
            active = self.active_manipulation
            self.active_manipulation = None
            self._manip_status(active, ManipulationStatus.STATE_CANCELLED, "cancelled")
        return response

    def _classify_sorting(self, request, response):
        del request
        response.success = True
        response.arena = self.get_parameter("arena").value
        response.park_1_cargo = "gear"
        response.park_2_cargo = "t_bolt"
        response.message = "mock sorting rule"
        return response

    def _detect(self, request, response):
        response.success = True
        response.layer_ok = True
        response.complete = True
        response.message = "verified"
        if request.waypoint_id == "pickup":
            response.cargo_class = (
                "gear" if (int(request.slot) + int(request.expected_layer)) % 2 == 0
                else "t_bolt"
            )
            response.target_count = 1
        else:
            response.cargo_class = ""
            response.target_count = 0
        return response

    def _publish_cmd_vel(self):
        if self.active_navigation is None:
            return
        cmd = Twist()
        cmd.linear.x = 0.10
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
