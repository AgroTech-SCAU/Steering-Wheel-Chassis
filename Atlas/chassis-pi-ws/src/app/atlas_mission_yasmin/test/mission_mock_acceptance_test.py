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

"""End-to-end acceptance tests for the Atlas competition YASMIN mock stack."""

import os
import signal
import subprocess
import threading
import time

import pytest
import rclpy
from atlas_mission_interfaces.msg import (
    ManipulationStatus,
    MissionStatus,
    NavigationStatus,
)
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor


SCENARIOS = [
    "normal",
    "navigation_failed",
    "reset_during_navigation",
    "fault_during_navigation",
    "estop_during_navigation",
    "mcu_timeout_during_navigation",
    "stale_navigation_status",
]

ROS_DOMAIN_BASE = int(os.environ.get("ATLAS_MOCK_TEST_ROS_DOMAIN_BASE", "180"))


class MockLaunch:
    def __init__(self, scenario):
        self.scenario = scenario
        self.domain_id = str(ROS_DOMAIN_BASE + SCENARIOS.index(scenario))
        self.output = []
        self.process = None
        self._reader_thread = None

    def start(self):
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = self.domain_id
        env["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
        env["ROS_LOG_DIR"] = "/tmp/atlas_ros_log"
        command = [
            "ros2",
            "launch",
            "atlas_mission_yasmin",
            "mission_yasmin_mock.launch.py",
            f"scenario:={self.scenario}",
        ]
        self.process = subprocess.Popen(
            command,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        self._reader_thread = threading.Thread(target=self._read_output, daemon=True)
        self._reader_thread.start()

    def stop(self):
        if self.process is None:
            return
        if self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGTERM)
            try:
                self.process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait(timeout=3.0)
        if self._reader_thread is not None:
            self._reader_thread.join(timeout=2.0)

    def _read_output(self):
        assert self.process is not None
        assert self.process.stdout is not None
        for line in self.process.stdout:
            self.output.append(line)

    def fail(self, message):
        tail = "".join(self.output[-120:])
        pytest.fail(f"{message}\n\nlaunch output tail:\n{tail}")


class MissionMockHarness:
    def __init__(self, mock_launch):
        os.environ["ROS_DOMAIN_ID"] = mock_launch.domain_id
        os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
        self.mock_launch = mock_launch
        self.context = rclpy.context.Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node("mission_mock_acceptance_test", context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)

        self.states = []
        self.mission_statuses = []
        self.nav_statuses = []
        self.manip_statuses = []
        self.motor_cmds = []
        self._stop_spin = threading.Event()

        self.status_sub = self.node.create_subscription(
            MissionStatus, "/atlas/mission/status", self._on_status, 100)
        self.nav_sub = self.node.create_subscription(
            NavigationStatus, "/atlas/navigation/status", self._on_nav_status, 100)
        self.manip_sub = self.node.create_subscription(
            ManipulationStatus, "/atlas/manipulation/status", self._on_manip_status, 100)
        self.motor_sub = self.node.create_subscription(
            Twist, "/motor_cmd_vel", self._on_motor_cmd, 100)

        self.spin_thread = threading.Thread(target=self._spin, daemon=True)
        self.spin_thread.start()

    def close(self):
        self._stop_spin.set()
        self.spin_thread.join(timeout=2.0)
        self.executor.remove_node(self.node)
        self.executor.shutdown()
        self.node.destroy_subscription(self.status_sub)
        self.node.destroy_subscription(self.nav_sub)
        self.node.destroy_subscription(self.manip_sub)
        self.node.destroy_subscription(self.motor_sub)
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)

    def _spin(self):
        while rclpy.ok(context=self.context) and not self._stop_spin.is_set():
            self.executor.spin_once(timeout_sec=0.05)

    def _on_status(self, msg):
        self.mission_statuses.append(msg)
        self.states.append(msg.state_name)

    def _on_nav_status(self, msg):
        self.nav_statuses.append(msg)

    def _on_manip_status(self, msg):
        self.manip_statuses.append(msg)

    def _on_motor_cmd(self, msg):
        self.motor_cmds.append(msg)

    def wait_until(self, predicate, timeout_s=20.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if predicate():
                return
            if (
                self.mock_launch.process is not None
                and self.mock_launch.process.poll() is not None
            ):
                self.mock_launch.fail(
                    "mock launch exited before acceptance condition was met")
            time.sleep(0.05)
        self.mock_launch.fail("acceptance condition was not met before timeout")

    def wait_for_state(self, state_name, timeout_s=20.0):
        self.wait_until(lambda: state_name in self.states, timeout_s)

    def nav_waypoints(self, state=NavigationStatus.STATE_RUNNING):
        return [msg.waypoint_id for msg in self.nav_statuses if msg.state == state]

    def manip_actions(self, state=ManipulationStatus.STATE_RUNNING):
        return [
            (msg.waypoint_id, msg.task_id)
            for msg in self.manip_statuses
            if msg.state == state
        ]

    def assert_final_motor_zero(self):
        self.wait_until(lambda: bool(self.motor_cmds), timeout_s=3.0)
        last = self.motor_cmds[-1]
        assert last.linear.x == 0.0
        assert last.linear.y == 0.0
        assert last.angular.z == 0.0


@pytest.mark.parametrize("scenario", SCENARIOS)
def test_mission_mock_acceptance(scenario):
    mock_launch = MockLaunch(scenario)
    harness = MissionMockHarness(mock_launch)
    try:
        mock_launch.start()
        if scenario == "normal":
            assert_normal_completion(harness)
        elif scenario == "navigation_failed":
            assert_navigation_failure(harness)
        elif scenario == "reset_during_navigation":
            assert_reset_interrupt(harness)
        elif scenario in ("fault_during_navigation", "estop_during_navigation"):
            assert_recovery_interrupt(harness)
        elif scenario == "mcu_timeout_during_navigation":
            assert_mcu_timeout(harness)
        elif scenario == "stale_navigation_status":
            assert_stale_navigation_status_ignored(harness)
        else:
            pytest.fail(f"unexpected scenario {scenario}")
    finally:
        mock_launch.stop()
        harness.close()


def assert_normal_completion(harness):
    harness.wait_for_state("WAIT_RESET")

    assert "INSPECT_SORT_ZONE" in harness.states
    assert "OBSERVE_PICKUP" in harness.states
    assert "PICK" in harness.states
    assert "OBSERVE_PARK" in harness.states
    assert "PLACE" in harness.states
    assert "CHECK_DONE" in harness.states
    assert "REPORT_DONE" in harness.states

    expected_nav = [
        "pickup", "park_1",
        "pickup", "park_2",
        "pickup", "park_2",
        "pickup", "park_1",
        "pickup", "park_1",
        "pickup", "park_2",
        "pickup", "park_2",
        "pickup", "park_1",
    ]
    assert harness.nav_waypoints() == expected_nav

    manip_actions = harness.manip_actions()
    assert ("sorting", "pre_recognition") not in manip_actions
    assert sum(task == "pick" for _, task in manip_actions) == 8
    assert sum(task == "place" for _, task in manip_actions) == 8

    assert all(
        waypoint in {"pickup", "park_1", "park_2"}
        for waypoint in harness.nav_waypoints()
    )
    harness.assert_final_motor_zero()


def assert_navigation_failure(harness):
    harness.wait_for_state("WAIT_RESET")
    assert harness.nav_waypoints() == ["pickup"]
    assert any(
        msg.waypoint_id == "pickup" and msg.state == NavigationStatus.STATE_FAILED
        for msg in harness.nav_statuses
    )
    assert "REPORT_FAIL" in harness.states
    assert "REPORT_DONE" not in harness.states
    harness.assert_final_motor_zero()


def assert_reset_interrupt(harness):
    harness.wait_until(
        lambda: any(
            msg.state == NavigationStatus.STATE_CANCELLED
            for msg in harness.nav_statuses
        )
    )
    time.sleep(0.3)
    assert harness.nav_waypoints() == ["pickup"]
    assert "REPORT_DONE" not in harness.states
    assert "REPORT_FAIL" not in harness.states
    assert "WAIT_RESET" in harness.states
    harness.assert_final_motor_zero()


def assert_recovery_interrupt(harness):
    harness.wait_for_state("RECOVERY")
    harness.wait_until(
        lambda: any(
            msg.state == NavigationStatus.STATE_CANCELLED
            for msg in harness.nav_statuses
        )
    )
    assert harness.nav_waypoints() == ["pickup"]
    assert "WAIT_RESET" in harness.states
    assert "REPORT_DONE" not in harness.states
    assert "REPORT_FAIL" not in harness.states
    harness.assert_final_motor_zero()


def assert_mcu_timeout(harness):
    # MCU timeout is detected by the mission guard and must enter recovery.
    # The navigation backend is not required to publish a cancel terminal state:
    # timeout can happen before the cancel service round trip completes.
    harness.wait_for_state("RECOVERY")
    assert harness.nav_waypoints() == ["pickup"]
    assert "WAIT_RESET" in harness.states
    assert "REPORT_DONE" not in harness.states
    assert "REPORT_FAIL" not in harness.states
    harness.assert_final_motor_zero()


def assert_stale_navigation_status_ignored(harness):
    harness.wait_for_state("OBSERVE_PICKUP")

    pickup_success = [
        msg for msg in harness.nav_statuses
        if msg.waypoint_id == "pickup"
        and msg.state == NavigationStatus.STATE_SUCCEEDED
    ]
    assert len(pickup_success) >= 2
    assert pickup_success[0].message == "stale terminal"
    assert pickup_success[1].message == "succeeded"

    observe_status = next(
        msg for msg in harness.mission_statuses
        if msg.state_name == "OBSERVE_PICKUP"
    )
    real_success_ns = (
        pickup_success[1].header.stamp.sec * 1_000_000_000
        + pickup_success[1].header.stamp.nanosec
    )
    observe_ns = (
        observe_status.header.stamp.sec * 1_000_000_000
        + observe_status.header.stamp.nanosec
    )
    assert real_success_ns <= observe_ns

    harness.wait_for_state("WAIT_RESET")
    assert "REPORT_DONE" in harness.states
    harness.assert_final_motor_zero()
