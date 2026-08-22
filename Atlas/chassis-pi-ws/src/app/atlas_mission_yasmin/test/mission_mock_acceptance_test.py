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

"""End-to-end acceptance tests for the Atlas YASMIN mock launch stack."""

import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

from ament_index_python.packages import get_package_share_directory
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy

from atlas_mission_interfaces.msg import MissionStatus
from geometry_msgs.msg import Twist
from std_msgs.msg import String


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
        share_dir = Path(get_package_share_directory("atlas_mission_yasmin"))
        config_file = share_dir / "config" / "mission_yasmin.yaml"
        route_file = share_dir / "config" / "mission_route.yaml"
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = self.domain_id
        env["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
        env["ROS_LOG_DIR"] = "/tmp/atlas_ros_log"
        command = [
            "ros2",
            "launch",
            "atlas_mission_yasmin",
            "mission_yasmin_mock.launch.py",
            f"config_file:={config_file}",
            f"route_file:={route_file}",
            f"scenario:={self.scenario}",
            "mcu_status_timeout_s:=0.3",
            "service_timeout_s:=0.5",
            "manipulation_result_timeout_s:=0.5",
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
        self.mock_launch = mock_launch
        self.context = rclpy.context.Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node("mission_mock_acceptance_test", context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.traces = []
        self.states = []
        self.motor_cmds = []
        self._stop_spin = threading.Event()

        trace_qos = QoSProfile(depth=100)
        trace_qos.reliability = ReliabilityPolicy.RELIABLE
        trace_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.trace_sub = self.node.create_subscription(
            String, "/atlas/mission_yasmin_mock/trace", self._on_trace, trace_qos)
        self.status_sub = self.node.create_subscription(
            MissionStatus, "/atlas/mission/status", self._on_status, 100)
        self.motor_sub = self.node.create_subscription(
            Twist, "/motor_cmd_vel", self._on_motor_cmd, 100)
        self.spin_thread = threading.Thread(target=self._spin, daemon=True)
        self.spin_thread.start()

    def close(self):
        self._stop_spin.set()
        self.spin_thread.join(timeout=2.0)
        self.executor.remove_node(self.node)
        self.executor.shutdown()
        self.node.destroy_subscription(self.trace_sub)
        self.node.destroy_subscription(self.status_sub)
        self.node.destroy_subscription(self.motor_sub)
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)

    def _spin(self):
        while rclpy.ok(context=self.context) and not self._stop_spin.is_set():
            self.executor.spin_once(timeout_sec=0.05)

    def _on_trace(self, msg):
        self.traces.append(json.loads(msg.data))

    def _on_status(self, msg):
        self.states.append(msg.state_name)

    def _on_motor_cmd(self, msg):
        self.motor_cmds.append(msg)

    def wait_until(self, predicate, timeout_s=12.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if predicate():
                return
            if (
                self.mock_launch.process is not None and
                self.mock_launch.process.poll() is not None
            ):
                self.mock_launch.fail("mock launch exited before acceptance condition was met")
            time.sleep(0.05)
        self.mock_launch.fail("acceptance condition was not met before timeout")

    def events(self, event_name):
        return [trace for trace in self.traces if trace.get("event") == event_name]

    def wait_for_trace(self, event_name, timeout_s=12.0):
        self.wait_until(lambda: bool(self.events(event_name)), timeout_s)

    def assert_final_motor_zero(self):
        self.wait_until(lambda: bool(self.motor_cmds), timeout_s=3.0)
        last = self.motor_cmds[-1]
        assert last.linear.x == 0.0
        assert last.linear.y == 0.0
        assert last.angular.z == 0.0

    def report_results(self):
        return [event.get("result") for event in self.events("report_result")]

    def trace_index(self, event_name, **fields):
        for index, event in enumerate(self.traces):
            if event.get("event") != event_name:
                continue
            if all(event.get(key) == value for key, value in fields.items()):
                return index
        self.mock_launch.fail(f"trace event {event_name} with {fields} was not observed")


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
            assert_recovery_interrupt(harness, scenario)
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
    harness.wait_for_trace("report_result")
    assert [event["waypoint_id"] for event in harness.events("nav_start")] == [
        "row_1_entry",
        "plant_1",
        "row_1_exit",
    ]
    assert [event["task_id"] for event in harness.events("manip_start")] == [
        "prepare_arm",
        "pollination",
    ]
    assert harness.report_results() == ["DONE"]
    assert "REPORT_DONE" in harness.states
    assert "WAIT_RESET" in harness.states
    harness.assert_final_motor_zero()


def assert_navigation_failure(harness):
    harness.wait_for_trace("report_result")
    assert [event["waypoint_id"] for event in harness.events("nav_start")] == ["row_1_entry"]
    assert harness.report_results() == ["FAIL"]
    assert "REPORT_FAIL" in harness.states
    assert "WAIT_RESET" in harness.states
    harness.assert_final_motor_zero()


def assert_reset_interrupt(harness):
    harness.wait_for_trace("nav_cancel")
    time.sleep(0.3)
    assert harness.report_results() == []
    assert "plant_1" not in [event["waypoint_id"] for event in harness.events("nav_start")]
    assert "WAIT_RESET" in harness.states
    harness.assert_final_motor_zero()


def assert_recovery_interrupt(harness, scenario):
    expected_event = "mcu_fault" if scenario == "fault_during_navigation" else "mcu_estop"
    harness.wait_for_trace(expected_event)
    harness.wait_for_trace("nav_cancel")
    time.sleep(0.3)
    assert harness.report_results() == []
    assert "plant_1" not in [event["waypoint_id"] for event in harness.events("nav_start")]
    assert "RECOVERY" in harness.states
    assert "WAIT_RESET" in harness.states
    harness.assert_final_motor_zero()


def assert_mcu_timeout(harness):
    harness.wait_for_trace("mcu_timeout_started")
    harness.wait_for_trace("nav_cancel")
    time.sleep(0.3)
    assert harness.report_results() == []
    assert "plant_1" not in [event["waypoint_id"] for event in harness.events("nav_start")]
    assert "RECOVERY" in harness.states
    assert "WAIT_RESET" in harness.states
    harness.assert_final_motor_zero()


def assert_stale_navigation_status_ignored(harness):
    harness.wait_until(lambda: "WAIT_RESET" in harness.states)
    assert "REPORT_DONE" in harness.states
    stale_index = harness.trace_index("nav_stale_terminal", waypoint_id="row_1_entry")
    succeeded_index = harness.trace_index("nav_succeeded", waypoint_id="row_1_entry")
    assert stale_index < succeeded_index
    assert [event["waypoint_id"] for event in harness.events("nav_start")] == [
        "row_1_entry",
        "plant_1",
        "row_1_exit",
    ]
    harness.assert_final_motor_zero()
