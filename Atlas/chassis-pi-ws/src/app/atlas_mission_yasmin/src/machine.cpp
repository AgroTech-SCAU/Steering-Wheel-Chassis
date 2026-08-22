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

yasmin::StateMachine::SharedPtr build_route_machine(const Runtime::SharedPtr & runtime)
{
  auto route = yasmin::StateMachine::make_shared(
    yasmin::Outcomes(
    {
      outcomes::kRouteDone,
      outcomes::kFailed,
      outcomes::kReset,
      outcomes::kRecovery,
      outcomes::kShutdown,
    }),
    false);

  route->add_state(
    "PREPARE_WAYPOINT",
    std::make_shared<PrepareWaypointState>(runtime),
    {
      {outcomes::kOk, "PRE_MOVE"},
      {outcomes::kRouteDone, outcomes::kRouteDone},
    });

  route->add_state(
    "PRE_MOVE",
    std::make_shared<PreMoveState>(runtime),
    {
      {outcomes::kOk, "NAVIGATE"},
      {outcomes::kFailed, outcomes::kFailed},
      {outcomes::kReset, outcomes::kReset},
      {outcomes::kRecovery, outcomes::kRecovery},
      {outcomes::kShutdown, outcomes::kShutdown},
    });

  route->add_state(
    "NAVIGATE",
    std::make_shared<NavigateState>(runtime),
    {
      {outcomes::kOk, "RUN_JOBS"},
      {outcomes::kFailed, outcomes::kFailed},
      {outcomes::kReset, outcomes::kReset},
      {outcomes::kRecovery, outcomes::kRecovery},
      {outcomes::kShutdown, outcomes::kShutdown},
    });

  route->add_state(
    "RUN_JOBS",
    std::make_shared<RunJobsState>(runtime),
    {
      {outcomes::kOk, "ADVANCE"},
      {outcomes::kFailed, outcomes::kFailed},
      {outcomes::kReset, outcomes::kReset},
      {outcomes::kRecovery, outcomes::kRecovery},
      {outcomes::kShutdown, outcomes::kShutdown},
    });

  route->add_state(
    "ADVANCE",
    std::make_shared<AdvanceState>(runtime),
    {
      {outcomes::kNext, "PREPARE_WAYPOINT"},
      {outcomes::kRouteDone, outcomes::kRouteDone},
    });

  return route;
}

yasmin::StateMachine::SharedPtr build_machine(const Runtime::SharedPtr & runtime)
{
  auto machine = yasmin::StateMachine::make_shared(
    yasmin::Outcomes({outcomes::kShutdown}),
    true);

  machine->add_state(
    "BOOTSTRAP",
    std::make_shared<BootstrapState>(runtime),
    {
      {outcomes::kOk, "WAIT_MCU"},
      {outcomes::kRecovery, "RECOVERY"},
      {outcomes::kShutdown, outcomes::kShutdown},
    });

  machine->add_state(
    "WAIT_MCU",
    std::make_shared<WaitMcuState>(runtime),
    {
      {outcomes::kOk, "WAIT_START"},
      {outcomes::kShutdown, outcomes::kShutdown},
    });

  machine->add_state(
    "WAIT_START",
    std::make_shared<WaitStartState>(runtime),
    {
      {outcomes::kOk, "PRECHECK"},
      {outcomes::kRecovery, "RECOVERY"},
      {outcomes::kShutdown, outcomes::kShutdown},
    });

  machine->add_state(
    "PRECHECK",
    std::make_shared<PrecheckState>(runtime),
    {
      {outcomes::kOk, "START_RUN"},
      {outcomes::kRetry, "PRECHECK"},
      {outcomes::kRecovery, "RECOVERY"},
      {outcomes::kShutdown, outcomes::kShutdown},
    });

  machine->add_state(
    "START_RUN",
    std::make_shared<StartRunState>(runtime),
    {
      {outcomes::kOk, "EXECUTE_ROUTE"},
    });

  machine->add_state(
    "EXECUTE_ROUTE",
    build_route_machine(runtime),
    {
      {outcomes::kRouteDone, "REPORT_DONE"},
      {outcomes::kFailed, "REPORT_FAIL"},
      {outcomes::kReset, "WAIT_RESET"},
      {outcomes::kRecovery, "RECOVERY"},
      {outcomes::kShutdown, outcomes::kShutdown},
    });

  machine->add_state(
    "REPORT_DONE",
    std::make_shared<ReportDoneState>(runtime),
    {
      {outcomes::kOk, "WAIT_RESET"},
      {outcomes::kRecovery, "RECOVERY"},
    });

  machine->add_state(
    "REPORT_FAIL",
    std::make_shared<ReportFailState>(runtime),
    {
      {outcomes::kOk, "WAIT_RESET"},
      {outcomes::kRecovery, "RECOVERY"},
    });

  machine->add_state(
    "RECOVERY",
    std::make_shared<RecoveryState>(runtime),
    {
      {outcomes::kOk, "WAIT_RESET"},
    });

  machine->add_state(
    "WAIT_RESET",
    std::make_shared<WaitResetState>(runtime),
    {
      {outcomes::kOk, "WAIT_MCU"},
      {outcomes::kShutdown, outcomes::kShutdown},
    });

  return machine;
}

}  // namespace atlas_mission_yasmin
