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

#include <string>

#include "atlas_mission_yasmin/machine.hpp"
#include "atlas_mission_yasmin/runtime.hpp"
#include "yasmin/types.hpp"

namespace
{

class MachineUnitTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    runtime = std::make_shared<atlas_mission_yasmin::Runtime>();
  }

  void TearDown() override
  {
    runtime.reset();
    rclcpp::shutdown();
  }

  atlas_mission_yasmin::Runtime::SharedPtr runtime;
};

void expect_transition(
  const yasmin::TransitionsMap & transitions,
  const std::string & state,
  const yasmin::Transitions & expected)
{
  ASSERT_TRUE(transitions.find(state) != transitions.end()) << state;
  EXPECT_EQ(transitions.at(state), expected) << state;
}

}  // namespace

TEST_F(MachineUnitTest, RouteMachineHasFrozenTopology)
{
  auto route = atlas_mission_yasmin::build_route_machine(runtime);
  ASSERT_NO_THROW(route->validate(true));

  EXPECT_EQ(route->get_start_state(), "PREPARE_WAYPOINT");
  EXPECT_EQ(
    route->get_outcomes(), yasmin::Outcomes(
  {
    "failed", "recovery", "reset", "route_done", "shutdown"}));

  const auto & transitions = route->get_transitions();
  EXPECT_EQ(transitions.size(), 5u);
  expect_transition(
    transitions, "PREPARE_WAYPOINT", {
    {"ok", "PRE_MOVE"},
    {"route_done", "route_done"},
  });
  expect_transition(
    transitions, "PRE_MOVE", {
    {"failed", "failed"},
    {"ok", "NAVIGATE"},
    {"recovery", "recovery"},
    {"reset", "reset"},
    {"shutdown", "shutdown"},
  });
  expect_transition(
    transitions, "NAVIGATE", {
    {"failed", "failed"},
    {"ok", "RUN_JOBS"},
    {"recovery", "recovery"},
    {"reset", "reset"},
    {"shutdown", "shutdown"},
  });
  expect_transition(
    transitions, "RUN_JOBS", {
    {"failed", "failed"},
    {"ok", "ADVANCE"},
    {"recovery", "recovery"},
    {"reset", "reset"},
    {"shutdown", "shutdown"},
  });
  expect_transition(
    transitions, "ADVANCE", {
    {"next", "PREPARE_WAYPOINT"},
    {"route_done", "route_done"},
  });
}

TEST_F(MachineUnitTest, RootMachineHasFrozenLifecycleTopology)
{
  auto machine = atlas_mission_yasmin::build_machine(runtime);
  ASSERT_NO_THROW(machine->validate(true));

  EXPECT_EQ(machine->get_start_state(), "BOOTSTRAP");
  EXPECT_EQ(machine->get_outcomes(), yasmin::Outcomes({"shutdown"}));

  const auto & transitions = machine->get_transitions();
  EXPECT_EQ(transitions.size(), 10u);
  expect_transition(
    transitions, "BOOTSTRAP", {
    {"ok", "WAIT_MCU"},
    {"recovery", "RECOVERY"},
    {"shutdown", "shutdown"},
  });
  expect_transition(
    transitions, "WAIT_MCU", {
    {"ok", "WAIT_START"},
    {"shutdown", "shutdown"},
  });
  expect_transition(
    transitions, "WAIT_START", {
    {"ok", "PRECHECK"},
    {"recovery", "RECOVERY"},
    {"shutdown", "shutdown"},
  });
  expect_transition(
    transitions, "PRECHECK", {
    {"ok", "START_RUN"},
    {"recovery", "RECOVERY"},
    {"retry", "PRECHECK"},
    {"shutdown", "shutdown"},
  });
  expect_transition(
    transitions, "START_RUN", {
    {"ok", "EXECUTE_ROUTE"},
  });
  expect_transition(
    transitions, "EXECUTE_ROUTE", {
    {"failed", "REPORT_FAIL"},
    {"recovery", "RECOVERY"},
    {"reset", "WAIT_RESET"},
    {"route_done", "REPORT_DONE"},
    {"shutdown", "shutdown"},
  });
  expect_transition(
    transitions, "REPORT_DONE", {
    {"ok", "WAIT_RESET"},
    {"recovery", "RECOVERY"},
  });
  expect_transition(
    transitions, "REPORT_FAIL", {
    {"ok", "WAIT_RESET"},
    {"recovery", "RECOVERY"},
  });
  expect_transition(
    transitions, "RECOVERY", {
    {"ok", "WAIT_RESET"},
  });
  expect_transition(
    transitions, "WAIT_RESET", {
    {"ok", "WAIT_MCU"},
    {"shutdown", "shutdown"},
  });
}

TEST_F(MachineUnitTest, RootMachineEmbedsRouteMachineWithMatchingOutcomes)
{
  auto machine = atlas_mission_yasmin::build_machine(runtime);
  const auto & states = machine->get_states();
  ASSERT_TRUE(states.find("EXECUTE_ROUTE") != states.end());

  const auto route = std::dynamic_pointer_cast<yasmin::StateMachine>(
    states.at("EXECUTE_ROUTE"));
  ASSERT_NE(route, nullptr);
  EXPECT_EQ(
    route->get_outcomes(), yasmin::Outcomes(
  {
    "failed", "recovery", "reset", "route_done", "shutdown"}));
  ASSERT_NO_THROW(route->validate(true));
}
