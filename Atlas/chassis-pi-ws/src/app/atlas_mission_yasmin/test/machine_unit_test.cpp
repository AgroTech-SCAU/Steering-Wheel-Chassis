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
}  // namespace

TEST_F(MachineUnitTest, AutonomousMachineHasCompetitionTopology)
{
  auto machine = atlas_mission_yasmin::build_autonomous_machine(runtime);
  ASSERT_NO_THROW(machine->validate(true));
  EXPECT_EQ(machine->get_start_state(), "INSPECT_SORT_ZONE");

  const auto & states = machine->get_states();
  EXPECT_EQ(states.size(), 8U);
  for (const auto * name : {
      "INSPECT_SORT_ZONE", "NAV_PICKUP", "OBSERVE_PICKUP", "PICK",
      "NAV_PARK", "OBSERVE_PARK", "PLACE", "CHECK_DONE"})
  {
    EXPECT_TRUE(states.find(name) != states.end()) << name;
  }
}

TEST_F(MachineUnitTest, RootWaitsForMcuAutoBeforeAutonomousMission)
{
  auto machine = atlas_mission_yasmin::build_machine(runtime);
  ASSERT_NO_THROW(machine->validate(true));
  const auto & transitions = machine->get_transitions();
  ASSERT_TRUE(transitions.find("WAIT_MCU") != transitions.end());
  ASSERT_TRUE(transitions.find("WAIT_AUTO") != transitions.end());
  EXPECT_EQ(transitions.at("WAIT_MCU").at("ok"), "WAIT_AUTO");
  EXPECT_EQ(transitions.at("START_RUN").at("ok"), "EXECUTE_AUTONOMOUS");
}
