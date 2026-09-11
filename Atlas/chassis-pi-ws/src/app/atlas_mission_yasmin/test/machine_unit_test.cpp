// Copyright 2026 yangxuan
// Licensed under the Apache License, Version 2.0

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

TEST_F(MachineUnitTest, AutonomousMachineUsesSafeArmKeyframeTopology)
{
  auto machine = atlas_mission_yasmin::build_autonomous_machine(runtime);
  ASSERT_NO_THROW(machine->validate(true));
  EXPECT_EQ(machine->get_start_state(), "ARM_ZERO");

  const auto & states = machine->get_states();
  EXPECT_EQ(states.size(), 14U);
  for (const auto * name : {
    "ARM_ZERO", "INSPECT_SORT_ZONE", "ARM_NAV_SAFE_INITIAL", "NAV_PICKUP",
    "OBSERVE_PICKUP", "PICK", "RETURN_PICKUP_OBSERVE", "ARM_NAV_SAFE_TO_PARK",
    "NAV_PARK", "PARK_PREPARE", "PLACE", "RETURN_PARK_PREPARE",
    "ARM_NAV_SAFE_TO_PICKUP", "CHECK_DONE"})
  {
    EXPECT_TRUE(states.find(name) != states.end()) << name;
  }

  const auto & transitions = machine->get_transitions();
  EXPECT_EQ(transitions.at("ARM_ZERO").at("ok"), "INSPECT_SORT_ZONE");
  EXPECT_EQ(transitions.at("INSPECT_SORT_ZONE").at("ok"), "ARM_NAV_SAFE_INITIAL");
  EXPECT_EQ(transitions.at("PICK").at("ok"), "RETURN_PICKUP_OBSERVE");
  EXPECT_EQ(transitions.at("RETURN_PICKUP_OBSERVE").at("ok"), "ARM_NAV_SAFE_TO_PARK");
  EXPECT_EQ(transitions.at("NAV_PARK").at("ok"), "PARK_PREPARE");
  EXPECT_EQ(transitions.at("PLACE").at("ok"), "RETURN_PARK_PREPARE");
  EXPECT_EQ(transitions.at("RETURN_PARK_PREPARE").at("ok"), "ARM_NAV_SAFE_TO_PICKUP");
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
