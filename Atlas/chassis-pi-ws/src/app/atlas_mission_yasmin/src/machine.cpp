// Copyright 2026 yangxuan
//
// Licensed under the Apache License, Version 2.0

#include "atlas_mission_yasmin/machine.hpp"

#include <memory>
#include <string>

#include "atlas_mission_yasmin/states.hpp"

namespace atlas_mission_yasmin
{

namespace
{
using TransitionMap = yasmin::Transitions;

TransitionMap action_transitions(const std::string & ok_target)
{
  return {
    {outcomes::kOk, ok_target},
    {outcomes::kFailed, outcomes::kFailed},
    {outcomes::kReset, outcomes::kReset},
    {outcomes::kRecovery, outcomes::kRecovery},
    {outcomes::kShutdown, outcomes::kShutdown},
  };
}
}  // namespace

yasmin::StateMachine::SharedPtr build_autonomous_machine(const Runtime::SharedPtr & runtime)
{
  auto machine = yasmin::StateMachine::make_shared(
    yasmin::Outcomes(
    {
      outcomes::kRouteDone,
      outcomes::kFailed,
      outcomes::kReset,
      outcomes::kRecovery,
      outcomes::kShutdown,
    }),
    false);

  machine->add_state(
    "ARM_ZERO", std::make_shared<ArmZeroState>(runtime),
    action_transitions("NAV_ORIGIN"));

  machine->add_state(
    "NAV_ORIGIN", std::make_shared<NavOriginState>(runtime),
    action_transitions("INSPECT_SORT_ZONE"));

  machine->add_state(
    "INSPECT_SORT_ZONE", std::make_shared<InspectSortZoneState>(runtime),
    action_transitions("ARM_NAV_SAFE_INITIAL"));

  machine->add_state(
    "ARM_NAV_SAFE_INITIAL", std::make_shared<ArmNavigationSafeState>(runtime),
    action_transitions("NAV_PICKUP"));

  machine->add_state(
    "NAV_PICKUP", std::make_shared<NavPickupState>(runtime),
    {
      {outcomes::kOk, "OBSERVE_PICKUP"},
      {outcomes::kRouteDone, outcomes::kRouteDone},
      {outcomes::kFailed, outcomes::kFailed},
      {outcomes::kReset, outcomes::kReset},
      {outcomes::kRecovery, outcomes::kRecovery},
      {outcomes::kShutdown, outcomes::kShutdown},
    });

  machine->add_state(
    "OBSERVE_PICKUP", std::make_shared<ObservePickupState>(runtime),
    action_transitions("PICK"));

  machine->add_state(
    "PICK", std::make_shared<PickState>(runtime),
    action_transitions("RETURN_PICKUP_OBSERVE"));

  machine->add_state(
    "RETURN_PICKUP_OBSERVE", std::make_shared<ReturnPickupObserveState>(runtime),
    action_transitions("ARM_NAV_SAFE_TO_PARK"));

  machine->add_state(
    "ARM_NAV_SAFE_TO_PARK", std::make_shared<ArmNavigationSafeState>(runtime),
    action_transitions("NAV_PARK"));

  machine->add_state(
    "NAV_PARK", std::make_shared<NavParkState>(runtime),
    action_transitions("PARK_PREPARE"));

  machine->add_state(
    "PARK_PREPARE", std::make_shared<ParkPrepareState>(runtime),
    action_transitions("PLACE"));

  machine->add_state(
    "PLACE", std::make_shared<PlaceState>(runtime),
    action_transitions("RETURN_PARK_PREPARE"));

  machine->add_state(
    "RETURN_PARK_PREPARE", std::make_shared<ParkPrepareState>(runtime),
    action_transitions("ARM_NAV_SAFE_TO_PICKUP"));

  machine->add_state(
    "ARM_NAV_SAFE_TO_PICKUP", std::make_shared<ArmNavigationSafeState>(runtime),
    action_transitions("CHECK_DONE"));

  machine->add_state(
    "CHECK_DONE", std::make_shared<CheckDoneState>(runtime),
    {
      {outcomes::kNext, "NAV_PICKUP"},
      {outcomes::kRouteDone, outcomes::kRouteDone},
    });

  return machine;
}

yasmin::StateMachine::SharedPtr build_machine(const Runtime::SharedPtr & runtime)
{
  auto machine = yasmin::StateMachine::make_shared(
    yasmin::Outcomes({outcomes::kShutdown}), true);

  machine->add_state(
    "BOOTSTRAP", std::make_shared<BootstrapState>(runtime),
    {
      {outcomes::kOk, "WAIT_MCU"},
      {outcomes::kShutdown, outcomes::kShutdown},
    });

  machine->add_state(
    "WAIT_MCU", std::make_shared<WaitMcuState>(runtime),
    {
      {outcomes::kOk, "WAIT_AUTO"},
      {outcomes::kReset, "WAIT_RESET"},
      {outcomes::kShutdown, outcomes::kShutdown},
    });

  machine->add_state(
    "WAIT_AUTO", std::make_shared<WaitAutoState>(runtime),
    {
      {outcomes::kOk, "PRECHECK"},
      {outcomes::kReset, "WAIT_RESET"},
      {outcomes::kRecovery, "RECOVERY"},
      {outcomes::kShutdown, outcomes::kShutdown},
    });

  machine->add_state(
    "PRECHECK", std::make_shared<PrecheckState>(runtime),
    {
      {outcomes::kOk, "START_RUN"},
      {outcomes::kReset, "WAIT_RESET"},
      {outcomes::kRecovery, "RECOVERY"},
      {outcomes::kShutdown, outcomes::kShutdown},
    });

  machine->add_state(
    "START_RUN", std::make_shared<StartRunState>(runtime),
    {{outcomes::kOk, "EXECUTE_AUTONOMOUS"}});

  machine->add_state(
    "EXECUTE_AUTONOMOUS", build_autonomous_machine(runtime),
    {
      {outcomes::kRouteDone, "REPORT_DONE"},
      {outcomes::kFailed, "REPORT_FAIL"},
      {outcomes::kReset, "WAIT_RESET"},
      {outcomes::kRecovery, "RECOVERY"},
      {outcomes::kShutdown, outcomes::kShutdown},
    });

  machine->add_state(
    "REPORT_DONE", std::make_shared<ReportDoneState>(runtime),
    {
      {outcomes::kOk, "WAIT_RESET"},
      {outcomes::kRecovery, "RECOVERY"},
    });

  machine->add_state(
    "REPORT_FAIL", std::make_shared<ReportFailState>(runtime),
    {
      {outcomes::kOk, "WAIT_RESET"},
      {outcomes::kRecovery, "RECOVERY"},
    });

  machine->add_state(
    "RECOVERY", std::make_shared<RecoveryState>(runtime),
    {{outcomes::kOk, "WAIT_RESET"}});

  machine->add_state(
    "WAIT_RESET", std::make_shared<WaitResetState>(runtime),
    {
      {outcomes::kOk, "WAIT_MCU"},
      {outcomes::kShutdown, outcomes::kShutdown},
    });

  return machine;
}

}  // namespace atlas_mission_yasmin
