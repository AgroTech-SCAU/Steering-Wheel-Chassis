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

#include "atlas_mission_yasmin/states.hpp"

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <utility>

#include "atlas_mission_interfaces/msg/mission_status.hpp"

namespace atlas_mission_yasmin
{

namespace
{

using MissionStatus = atlas_mission_interfaces::msg::MissionStatus;

std::string action_result_to_outcome(const ActionResult result)
{
  switch (result) {
    case ActionResult::kSucceeded:
      return outcomes::kOk;
    case ActionResult::kFailed:
    case ActionResult::kRejected:
    case ActionResult::kTimeout:
      return outcomes::kFailed;
    case ActionResult::kReset:
      return outcomes::kReset;
    case ActionResult::kRecovery:
      return outcomes::kRecovery;
    case ActionResult::kShutdown:
      return outcomes::kShutdown;
  }
  return outcomes::kShutdown;
}

std::string guard_result_to_precheck_outcome(const GuardResult result)
{
  switch (result) {
    case GuardResult::kOk:
      return outcomes::kOk;
    case GuardResult::kReset:
      return outcomes::kRetry;
    case GuardResult::kRecovery:
      return outcomes::kRecovery;
    case GuardResult::kShutdown:
      return outcomes::kShutdown;
  }
  return outcomes::kShutdown;
}

std::size_t waypoint_index(yasmin::Blackboard::SharedPtr blackboard)
{
  if (!blackboard->contains("waypoint_index")) {
    return 0;
  }
  return blackboard->get<std::size_t>("waypoint_index");
}

bool reset_origin(yasmin::Blackboard::SharedPtr blackboard)
{
  if (!blackboard->contains("reset_origin")) {
    return true;
  }
  return blackboard->get<bool>("reset_origin");
}

const Waypoint * current_waypoint(
  const Plan & plan,
  yasmin::Blackboard::SharedPtr blackboard)
{
  const auto index = waypoint_index(blackboard);
  if (index >= plan.waypoints.size()) {
    return nullptr;
  }
  return &plan.waypoints[index];
}

}  // namespace

RuntimeState::RuntimeState(Runtime::SharedPtr runtime, yasmin::Outcomes state_outcomes)
: yasmin::State(std::move(state_outcomes)),
  runtime_(std::move(runtime))
{
  if (!runtime_) {
    throw std::invalid_argument("RuntimeState requires a runtime");
  }
}

BootstrapState::BootstrapState(Runtime::SharedPtr runtime)
: RuntimeState(std::move(runtime), {outcomes::kOk, outcomes::kRecovery, outcomes::kShutdown})
{
}

std::string BootstrapState::execute(yasmin::Blackboard::SharedPtr blackboard)
{
  (void)blackboard;
  runtime_->set_state(MissionStatus::STATE_BOOTSTRAP, "BOOTSTRAP", "");
  return rclcpp::ok() ? outcomes::kOk : outcomes::kShutdown;
}

WaitMcuState::WaitMcuState(Runtime::SharedPtr runtime)
: RuntimeState(std::move(runtime), {outcomes::kOk, outcomes::kShutdown})
{
}

std::string WaitMcuState::execute(yasmin::Blackboard::SharedPtr blackboard)
{
  (void)blackboard;
  runtime_->set_state(MissionStatus::STATE_WAIT_MCU_STATUS, "WAIT_MCU", "");
  while (rclcpp::ok()) {
    const auto result = runtime_->wait_mcu();
    if (result == WaitResult::kSuccess) {
      return outcomes::kOk;
    }
    if (result == WaitResult::kShutdown) {
      return outcomes::kShutdown;
    }
  }
  return outcomes::kShutdown;
}

WaitStartState::WaitStartState(Runtime::SharedPtr runtime)
: RuntimeState(
    std::move(runtime),
    {outcomes::kOk, outcomes::kRecovery, outcomes::kShutdown})
{
}

std::string WaitStartState::execute(yasmin::Blackboard::SharedPtr blackboard)
{
  (void)blackboard;
  runtime_->set_state(MissionStatus::STATE_WAIT_START, "WAIT_START", "");
  while (rclcpp::ok()) {
    const auto result = runtime_->wait_start();
    if (result == WaitResult::kSuccess) {
      return outcomes::kOk;
    }
    if (result == WaitResult::kRecovery || result == WaitResult::kReset) {
      return outcomes::kRecovery;
    }
    if (result == WaitResult::kShutdown) {
      return outcomes::kShutdown;
    }
  }
  return outcomes::kShutdown;
}

PrecheckState::PrecheckState(Runtime::SharedPtr runtime)
: RuntimeState(
    std::move(runtime),
    {outcomes::kOk, outcomes::kRetry, outcomes::kRecovery, outcomes::kShutdown})
{
}

std::string PrecheckState::execute(yasmin::Blackboard::SharedPtr blackboard)
{
  (void)blackboard;
  runtime_->set_state(MissionStatus::STATE_PRECHECK, "PRECHECK", "");
  return guard_result_to_precheck_outcome(runtime_->guard());
}

StartRunState::StartRunState(Runtime::SharedPtr runtime)
: RuntimeState(std::move(runtime), {outcomes::kOk})
{
}

std::string StartRunState::execute(yasmin::Blackboard::SharedPtr blackboard)
{
  runtime_->begin_run();
  blackboard->set<std::size_t>("waypoint_index", 0U);
  blackboard->set<bool>("reset_origin", true);
  return outcomes::kOk;
}

ReportDoneState::ReportDoneState(Runtime::SharedPtr runtime)
: RuntimeState(std::move(runtime), {outcomes::kOk, outcomes::kRecovery})
{
}

std::string ReportDoneState::execute(yasmin::Blackboard::SharedPtr blackboard)
{
  (void)blackboard;
  runtime_->set_state(MissionStatus::STATE_REPORTING_DONE, "REPORT_DONE", "");
  runtime_->safe_stop("report done");
  return runtime_->report_done() ? outcomes::kOk : outcomes::kRecovery;
}

ReportFailState::ReportFailState(Runtime::SharedPtr runtime)
: RuntimeState(std::move(runtime), {outcomes::kOk, outcomes::kRecovery})
{
}

std::string ReportFailState::execute(yasmin::Blackboard::SharedPtr blackboard)
{
  runtime_->set_state(MissionStatus::STATE_REPORTING_FAIL, "REPORT_FAIL", "");
  runtime_->safe_stop("report fail");
  const auto code = blackboard->contains("error_code") ?
    static_cast<int16_t>(blackboard->get<int32_t>("error_code")) :
    static_cast<int16_t>(1);
  return runtime_->report_fail(code) ? outcomes::kOk : outcomes::kRecovery;
}

RecoveryState::RecoveryState(Runtime::SharedPtr runtime)
: RuntimeState(std::move(runtime), {outcomes::kOk})
{
}

std::string RecoveryState::execute(yasmin::Blackboard::SharedPtr blackboard)
{
  (void)blackboard;
  runtime_->set_state(MissionStatus::STATE_RECOVERY_REQUIRED, "RECOVERY", "");
  runtime_->safe_stop("recovery");
  runtime_->clear_run();
  return outcomes::kOk;
}

WaitResetState::WaitResetState(Runtime::SharedPtr runtime)
: RuntimeState(std::move(runtime), {outcomes::kOk, outcomes::kShutdown})
{
}

std::string WaitResetState::execute(yasmin::Blackboard::SharedPtr blackboard)
{
  (void)blackboard;
  runtime_->set_state(MissionStatus::STATE_WAIT_RESET, "WAIT_RESET", "");
  const auto result = runtime_->wait_reset();
  return result == WaitResult::kShutdown ? outcomes::kShutdown : outcomes::kOk;
}

PrepareWaypointState::PrepareWaypointState(Runtime::SharedPtr runtime)
: RuntimeState(std::move(runtime), {outcomes::kOk, outcomes::kRouteDone})
{
}

std::string PrepareWaypointState::execute(yasmin::Blackboard::SharedPtr blackboard)
{
  return current_waypoint(runtime_->plan(), blackboard) == nullptr ?
         outcomes::kRouteDone :
         outcomes::kOk;
}

PreMoveState::PreMoveState(Runtime::SharedPtr runtime)
: RuntimeState(
    std::move(runtime),
    {outcomes::kOk, outcomes::kFailed, outcomes::kReset, outcomes::kRecovery,
      outcomes::kShutdown})
{
}

std::string PreMoveState::execute(yasmin::Blackboard::SharedPtr blackboard)
{
  const auto * waypoint = current_waypoint(runtime_->plan(), blackboard);
  if (waypoint == nullptr) {
    return outcomes::kFailed;
  }
  if (waypoint->pre_move_action.empty() || waypoint->pre_move_action == "noop") {
    return outcomes::kOk;
  }
  return action_result_to_outcome(runtime_->run_pre_move(*waypoint));
}

NavigateState::NavigateState(Runtime::SharedPtr runtime)
: RuntimeState(
    std::move(runtime),
    {outcomes::kOk, outcomes::kFailed, outcomes::kReset, outcomes::kRecovery,
      outcomes::kShutdown})
{
}

std::string NavigateState::execute(yasmin::Blackboard::SharedPtr blackboard)
{
  const auto * waypoint = current_waypoint(runtime_->plan(), blackboard);
  if (waypoint == nullptr) {
    return outcomes::kFailed;
  }

  const auto result = runtime_->run_navigation(*waypoint, reset_origin(blackboard));
  if (result == ActionResult::kSucceeded) {
    blackboard->set<bool>("reset_origin", false);
  }
  return action_result_to_outcome(result);
}

RunJobsState::RunJobsState(Runtime::SharedPtr runtime)
: RuntimeState(
    std::move(runtime),
    {outcomes::kOk, outcomes::kFailed, outcomes::kReset, outcomes::kRecovery,
      outcomes::kShutdown})
{
}

std::string RunJobsState::execute(yasmin::Blackboard::SharedPtr blackboard)
{
  const auto * waypoint = current_waypoint(runtime_->plan(), blackboard);
  if (waypoint == nullptr) {
    return outcomes::kFailed;
  }

  for (const auto & job : waypoint->arrival_jobs) {
    const auto result = runtime_->run_job(*waypoint, job);
    if (result != ActionResult::kSucceeded) {
      return action_result_to_outcome(result);
    }
  }
  return outcomes::kOk;
}

AdvanceState::AdvanceState(Runtime::SharedPtr runtime)
: RuntimeState(std::move(runtime), {outcomes::kNext, outcomes::kRouteDone})
{
}

std::string AdvanceState::execute(yasmin::Blackboard::SharedPtr blackboard)
{
  const auto next_index = waypoint_index(blackboard) + 1U;
  blackboard->set<std::size_t>("waypoint_index", next_index);
  return next_index >= runtime_->plan().waypoints.size() ?
         outcomes::kRouteDone :
         outcomes::kNext;
}

}  // namespace atlas_mission_yasmin
