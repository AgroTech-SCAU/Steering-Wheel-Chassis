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

#include "atlas_mission_yasmin/machine.hpp"

#include <memory>

#include "atlas_mission_yasmin/states.hpp"

namespace atlas_mission_yasmin
{

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
    "INSPECT_SORT_ZONE", std::make_shared<InspectSortZoneState>(runtime),
    {
      {outcomes::kOk, "NAV_PICKUP"},
      {outcomes::kFailed, outcomes::kFailed},
      {outcomes::kReset, outcomes::kReset},
      {outcomes::kRecovery, outcomes::kRecovery},
      {outcomes::kShutdown, outcomes::kShutdown},
    });

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
    {
      {outcomes::kOk, "PICK"},
      {outcomes::kFailed, outcomes::kFailed},
      {outcomes::kReset, outcomes::kReset},
      {outcomes::kRecovery, outcomes::kRecovery},
      {outcomes::kShutdown, outcomes::kShutdown},
    });

  machine->add_state(
    "PICK", std::make_shared<PickState>(runtime),
    {
      {outcomes::kOk, "NAV_PARK"},
      {outcomes::kFailed, outcomes::kFailed},
      {outcomes::kReset, outcomes::kReset},
      {outcomes::kRecovery, outcomes::kRecovery},
      {outcomes::kShutdown, outcomes::kShutdown},
    });

  machine->add_state(
    "NAV_PARK", std::make_shared<NavParkState>(runtime),
    {
      {outcomes::kOk, "OBSERVE_PARK"},
      {outcomes::kFailed, outcomes::kFailed},
      {outcomes::kReset, outcomes::kReset},
      {outcomes::kRecovery, outcomes::kRecovery},
      {outcomes::kShutdown, outcomes::kShutdown},
    });

  machine->add_state(
    "OBSERVE_PARK", std::make_shared<ObserveParkState>(runtime),
    {
      {outcomes::kOk, "PLACE"},
      {outcomes::kFailed, outcomes::kFailed},
      {outcomes::kReset, outcomes::kReset},
      {outcomes::kRecovery, outcomes::kRecovery},
      {outcomes::kShutdown, outcomes::kShutdown},
    });

  machine->add_state(
    "PLACE", std::make_shared<PlaceState>(runtime),
    {
      {outcomes::kOk, "CHECK_DONE"},
      {outcomes::kFailed, outcomes::kFailed},
      {outcomes::kReset, outcomes::kReset},
      {outcomes::kRecovery, outcomes::kRecovery},
      {outcomes::kShutdown, outcomes::kShutdown},
    });

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
