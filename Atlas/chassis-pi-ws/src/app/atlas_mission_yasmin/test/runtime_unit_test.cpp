// Copyright 2026 yangxuan
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <chrono>
#include <fstream>
#include <string>
#include <thread>

#include "atlas_mission_yasmin/runtime.hpp"
#include "mcu_comm_bridge/msg/auto_task_event.hpp"
#include "mcu_comm_bridge/msg/mcu_status.hpp"

using namespace std::chrono_literals;

namespace
{

std::string write_route_file()
{
  const std::string path = "/tmp/atlas_mission_yasmin_runtime_route.yaml";
  std::ofstream out(path);
  out
    << "navigation_backend: pseudo\n"
    << "manipulation_backend: vision\n"
    << "return_home_enabled: true\n"
    << "waypoints:\n"
    << "  - id: wp_1\n"
    << "    x_m: 1.0\n"
    << "    y_m: 2.0\n"
    << "    yaw_rad: 0.3\n"
    << "    pre_move_action: align\n"
    << "    timeout_s: 12.5\n"
    << "    arrival_jobs:\n"
    << "      - pollinate\n"
    << "      - inspect\n"
    << "return_waypoints:\n"
    << "  - id: home\n"
    << "    x_m: 0.0\n"
    << "    y_m: 0.0\n"
    << "    yaw_rad: 0.0\n";
  return path;
}

mcu_comm_bridge::msg::McuStatus make_status(const uint8_t app_state)
{
  mcu_comm_bridge::msg::McuStatus status;
  status.app_state = app_state;
  status.ready_flags = 0xff;
  return status;
}

}  // namespace

TEST(RuntimeUnitTest, LoadsRoutePlanFromYaml)
{
  const auto plan = atlas_mission_yasmin::Runtime::load_plan(write_route_file());

  ASSERT_EQ(plan.navigation_backend, "pseudo");
  ASSERT_EQ(plan.manipulation_backend, "vision");
  ASSERT_TRUE(plan.return_home_enabled);
  ASSERT_EQ(plan.waypoints.size(), 1u);
  ASSERT_EQ(plan.return_waypoints.size(), 1u);
  EXPECT_EQ(plan.waypoints[0].id, "wp_1");
  EXPECT_DOUBLE_EQ(plan.waypoints[0].x_m, 1.0);
  EXPECT_EQ(plan.waypoints[0].pre_move_action, "align");
  ASSERT_EQ(plan.waypoints[0].arrival_jobs.size(), 2u);
  EXPECT_EQ(plan.waypoints[0].arrival_jobs[1].task_id, "inspect");
}

TEST(RuntimeUnitTest, McuFreshRequiresRecentStatus)
{
  rclcpp::init(0, nullptr);
  auto runtime = std::make_shared<atlas_mission_yasmin::Runtime>();
  runtime->set_mcu_status_timeout_for_test(0.01);
  runtime->inject_mcu_status_for_test(
    make_status(mcu_comm_bridge::msg::McuStatus::STATE_AUTO_PI));

  EXPECT_TRUE(runtime->mcu_fresh());
  std::this_thread::sleep_for(30ms);
  EXPECT_FALSE(runtime->mcu_fresh());
  rclcpp::shutdown();
}

TEST(RuntimeUnitTest, GuardRejectsFaultEstopManualAndReset)
{
  rclcpp::init(0, nullptr);
  auto runtime = std::make_shared<atlas_mission_yasmin::Runtime>();

  runtime->inject_mcu_status_for_test(
    make_status(mcu_comm_bridge::msg::McuStatus::STATE_AUTO_PI));
  EXPECT_EQ(runtime->guard(), atlas_mission_yasmin::GuardResult::kOk);

  runtime->inject_mcu_status_for_test(
    make_status(mcu_comm_bridge::msg::McuStatus::STATE_FAULT));
  EXPECT_EQ(runtime->guard(), atlas_mission_yasmin::GuardResult::kRecovery);

  runtime->inject_mcu_status_for_test(
    make_status(mcu_comm_bridge::msg::McuStatus::STATE_ESTOP));
  EXPECT_EQ(runtime->guard(), atlas_mission_yasmin::GuardResult::kRecovery);

  runtime->inject_mcu_status_for_test(
    make_status(mcu_comm_bridge::msg::McuStatus::STATE_MANUAL));
  EXPECT_EQ(runtime->guard(), atlas_mission_yasmin::GuardResult::kRecovery);

  mcu_comm_bridge::msg::AutoTaskEvent reset;
  reset.event = mcu_comm_bridge::msg::AutoTaskEvent::EVENT_RESET;
  runtime->inject_auto_task_event_for_test(reset);
  EXPECT_EQ(runtime->guard(), atlas_mission_yasmin::GuardResult::kReset);
  rclcpp::shutdown();
}

TEST(RuntimeUnitTest, SafeStopPublishesZeroVelocityAndDisablesMotion)
{
  rclcpp::init(0, nullptr);
  auto runtime = std::make_shared<atlas_mission_yasmin::Runtime>();
  runtime->begin_run();
  runtime->set_motion_enabled_for_test(true);

  runtime->safe_stop("unit test");

  EXPECT_FALSE(runtime->can_move());
  EXPECT_EQ(runtime->safe_stop_count_for_test(), 1u);
  rclcpp::shutdown();
}
