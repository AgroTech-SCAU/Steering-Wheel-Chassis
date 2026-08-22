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

#include <memory>

#include "atlas_mission_yasmin/runtime.hpp"
#include "atlas_mission_yasmin/states.hpp"
#include "mcu_comm_bridge/msg/mcu_status.hpp"
#include "yasmin/blackboard.hpp"

namespace
{

atlas_mission_yasmin::Plan make_plan()
{
  atlas_mission_yasmin::Plan plan;

  atlas_mission_yasmin::Waypoint first;
  first.id = "wp_1";
  first.pre_move_action = "noop";
  first.arrival_jobs.push_back(atlas_mission_yasmin::Job{"inspect"});

  atlas_mission_yasmin::Waypoint second;
  second.id = "wp_2";

  plan.waypoints = {first, second};
  return plan;
}

mcu_comm_bridge::msg::McuStatus auto_pi_status()
{
  mcu_comm_bridge::msg::McuStatus status;
  status.app_state = mcu_comm_bridge::msg::McuStatus::STATE_AUTO_PI;
  status.ready_flags = 0xff;
  return status;
}

class StatesUnitTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    runtime = std::make_shared<atlas_mission_yasmin::Runtime>();
    runtime->set_plan_for_test(make_plan());
    runtime->inject_mcu_status_for_test(auto_pi_status());
    blackboard = yasmin::Blackboard::make_shared();
  }

  void TearDown() override
  {
    runtime.reset();
    blackboard.reset();
    rclcpp::shutdown();
  }

  atlas_mission_yasmin::Runtime::SharedPtr runtime;
  yasmin::Blackboard::SharedPtr blackboard;
};

}  // namespace

TEST_F(StatesUnitTest, StartRunInitializesWaypointIndexAndResetOrigin)
{
  atlas_mission_yasmin::StartRunState state(runtime);

  EXPECT_EQ(state(blackboard), atlas_mission_yasmin::outcomes::kOk);
  EXPECT_EQ(blackboard->get<std::size_t>("waypoint_index"), 0u);
  EXPECT_TRUE(blackboard->get<bool>("reset_origin"));
}

TEST_F(StatesUnitTest, AdvanceMovesToNextWaypointThenReportsRouteDone)
{
  atlas_mission_yasmin::AdvanceState state(runtime);
  blackboard->set<std::size_t>("waypoint_index", 0u);

  EXPECT_EQ(state(blackboard), atlas_mission_yasmin::outcomes::kNext);
  EXPECT_EQ(blackboard->get<std::size_t>("waypoint_index"), 1u);
  EXPECT_EQ(state(blackboard), atlas_mission_yasmin::outcomes::kRouteDone);
  EXPECT_EQ(blackboard->get<std::size_t>("waypoint_index"), 2u);
}

TEST_F(StatesUnitTest, NavigatePassesResetOriginAndClearsItOnSuccess)
{
  atlas_mission_yasmin::NavigateState state(runtime);
  blackboard->set<std::size_t>("waypoint_index", 0u);
  blackboard->set<bool>("reset_origin", true);
  runtime->set_next_navigation_result_for_test(atlas_mission_yasmin::ActionResult::kSucceeded);

  EXPECT_EQ(state(blackboard), atlas_mission_yasmin::outcomes::kOk);
  EXPECT_FALSE(blackboard->get<bool>("reset_origin"));
  ASSERT_TRUE(runtime->last_navigation_reset_origin_for_test().has_value());
  EXPECT_TRUE(runtime->last_navigation_reset_origin_for_test().value());
}

TEST_F(StatesUnitTest, RunJobsStopsAtFirstNonSuccessOutcome)
{
  atlas_mission_yasmin::RunJobsState state(runtime);
  blackboard->set<std::size_t>("waypoint_index", 0u);
  runtime->set_next_job_result_for_test(atlas_mission_yasmin::ActionResult::kReset);

  EXPECT_EQ(state(blackboard), atlas_mission_yasmin::outcomes::kReset);
}

TEST_F(StatesUnitTest, WaitAndPrecheckStatesMapRuntimeResultsToOutcomes)
{
  EXPECT_EQ(
    atlas_mission_yasmin::WaitMcuState(runtime)(blackboard),
    atlas_mission_yasmin::outcomes::kOk);
  EXPECT_EQ(
    atlas_mission_yasmin::PrecheckState(runtime)(blackboard),
    atlas_mission_yasmin::outcomes::kOk);
}
