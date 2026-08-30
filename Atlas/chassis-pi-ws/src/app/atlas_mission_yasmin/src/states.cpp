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

std::string action_outcome(const ActionResult result)
{
  switch (result) {
    case ActionResult::kSucceeded:
      return outcomes::kOk;
    case ActionResult::kReset:
      return outcomes::kReset;
    case ActionResult::kRecovery:
      return outcomes::kRecovery;
    case ActionResult::kShutdown:
      return outcomes::kShutdown;
    case ActionResult::kFailed:
    case ActionResult::kRejected:
    case ActionResult::kTimeout:
      return outcomes::kFailed;
  }
  return outcomes::kFailed;
}

std::string guard_outcome(const GuardResult result)
{
  switch (result) {
    case GuardResult::kOk:
      return outcomes::kOk;
    case GuardResult::kReset:
      return outcomes::kReset;
    case GuardResult::kRecovery:
      return outcomes::kRecovery;
    case GuardResult::kShutdown:
      return outcomes::kShutdown;
  }
  return outcomes::kShutdown;
}

std::size_t get_slot(
  const yasmin::Blackboard::SharedPtr & blackboard,
  const std::string & key)
{
  if (!blackboard->contains(key)) {
    return CompetitionModel::kSlotCount;
  }
  return blackboard->get<std::size_t>(key);
}

}  // namespace

RuntimeState::RuntimeState(Runtime::SharedPtr runtime, yasmin::Outcomes state_outcomes)
: yasmin::State(std::move(state_outcomes)), runtime_(std::move(runtime))
{
  if (!runtime_) {
    throw std::invalid_argument("RuntimeState requires a runtime");
  }
}

BootstrapState::BootstrapState(Runtime::SharedPtr runtime)
: RuntimeState(std::move(runtime), {outcomes::kOk, outcomes::kShutdown})
{
}

std::string BootstrapState::execute(yasmin::Blackboard::SharedPtr blackboard)
{
  (void)blackboard;
  runtime_->set_state(MissionStatus::STATE_BOOTSTRAP, "BOOTSTRAP", "");
  return rclcpp::ok() ? outcomes::kOk : outcomes::kShutdown;
}

WaitMcuState::WaitMcuState(Runtime::SharedPtr runtime)
: RuntimeState(std::move(runtime), {outcomes::kOk, outcomes::kReset, outcomes::kShutdown})
{
}

std::string WaitMcuState::execute(yasmin::Blackboard::SharedPtr blackboard)
{
  (void)blackboard;
  runtime_->set_state(MissionStatus::STATE_WAIT_MCU_STATUS, "WAIT_MCU", "");
  while (rclcpp::ok() && !is_canceled()) {
    const auto result = runtime_->wait_mcu();
    if (result == WaitResult::kSuccess) {
      return outcomes::kOk;
    }
    if (result == WaitResult::kReset) {
      return outcomes::kReset;
    }
    if (result == WaitResult::kShutdown) {
      return outcomes::kShutdown;
    }
  }
  return outcomes::kShutdown;
}

WaitAutoState::WaitAutoState(Runtime::SharedPtr runtime)
: RuntimeState(
    std::move(runtime),
    {outcomes::kOk, outcomes::kReset, outcomes::kRecovery, outcomes::kShutdown})
{
}

std::string WaitAutoState::execute(yasmin::Blackboard::SharedPtr blackboard)
{
  (void)blackboard;
  runtime_->set_state(MissionStatus::STATE_WAIT_START, "WAIT_AUTO", "waiting MCU AUTO event");
  while (rclcpp::ok() && !is_canceled()) {
    const auto result = runtime_->wait_auto();
    if (result == WaitResult::kSuccess) {
      return outcomes::kOk;
    }
    if (result == WaitResult::kReset) {
      return outcomes::kReset;
    }
    if (result == WaitResult::kRecovery) {
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
    {outcomes::kOk, outcomes::kReset, outcomes::kRecovery, outcomes::kShutdown})
{
}

std::string PrecheckState::execute(yasmin::Blackboard::SharedPtr blackboard)
{
  (void)blackboard;
  runtime_->set_state(MissionStatus::STATE_PRECHECK, "PRECHECK", "");
  return guard_outcome(runtime_->guard());
}

StartRunState::StartRunState(Runtime::SharedPtr runtime)
: RuntimeState(std::move(runtime), {outcomes::kOk})
{
}

std::string StartRunState::execute(yasmin::Blackboard::SharedPtr blackboard)
{
  runtime_->begin_run();
  blackboard->set<std::string>("cargo", "");
  blackboard->set<std::string>("destination", "");
  return outcomes::kOk;
}

InspectSortZoneState::InspectSortZoneState(Runtime::SharedPtr runtime)
: RuntimeState(
    std::move(runtime),
    {outcomes::kOk, outcomes::kFailed, outcomes::kReset, outcomes::kRecovery,
      outcomes::kShutdown})
{
}

std::string InspectSortZoneState::execute(yasmin::Blackboard::SharedPtr blackboard)
{
  runtime_->set_state(MissionStatus::STATE_RUNNING, "INSPECT_SORT_ZONE", "");
  const auto result = runtime_->inspect_sorting_zone();
  if (result.result == ActionResult::kSucceeded) {
    blackboard->set<std::string>("arena", result.arena);
    return outcomes::kOk;
  }
  return action_outcome(result.result);
}

NavPickupState::NavPickupState(Runtime::SharedPtr runtime)
: RuntimeState(
    std::move(runtime),
    {outcomes::kOk, outcomes::kRouteDone, outcomes::kFailed, outcomes::kReset,
      outcomes::kRecovery, outcomes::kShutdown})
{
}

std::string NavPickupState::execute(yasmin::Blackboard::SharedPtr blackboard)
{
  runtime_->set_state(MissionStatus::STATE_RUNNING, "NAV_PICKUP", "");
  if (runtime_->model().done()) {
    return outcomes::kRouteDone;
  }
  const auto slot = runtime_->model().next_pickup_slot();
  if (slot >= CompetitionModel::kSlotCount) {
    return outcomes::kFailed;
  }
  blackboard->set<std::size_t>("pickup_slot", slot);
  return action_outcome(runtime_->navigate("pickup"));
}

ObservePickupState::ObservePickupState(Runtime::SharedPtr runtime)
: RuntimeState(
    std::move(runtime),
    {outcomes::kOk, outcomes::kFailed, outcomes::kReset, outcomes::kRecovery,
      outcomes::kShutdown})
{
}

std::string ObservePickupState::execute(yasmin::Blackboard::SharedPtr blackboard)
{
  runtime_->set_state(MissionStatus::STATE_RUNNING, "OBSERVE_PICKUP", "");
  const auto slot = get_slot(blackboard, "pickup_slot");
  if (slot >= CompetitionModel::kSlotCount) {
    return outcomes::kFailed;
  }
  const auto layer = runtime_->model().pickup_layer(slot);
  const auto observation = runtime_->observe_with_recovery("pickup", slot, layer);
  if (observation.result != ActionResult::kSucceeded) {
    return action_outcome(observation.result);
  }
  const auto & destination = runtime_->model().destination_for(observation.cargo_class);
  if (destination.empty()) {
    return outcomes::kFailed;
  }
  const auto park_slot = runtime_->model().next_park_slot(destination);
  if (park_slot >= CompetitionModel::kSlotCount) {
    return outcomes::kFailed;
  }
  blackboard->set<std::string>("cargo", observation.cargo_class);
  blackboard->set<std::string>("destination", destination);
  blackboard->set<std::size_t>("park_slot", park_slot);
  return outcomes::kOk;
}

PickState::PickState(Runtime::SharedPtr runtime)
: RuntimeState(
    std::move(runtime),
    {outcomes::kOk, outcomes::kFailed, outcomes::kReset, outcomes::kRecovery,
      outcomes::kShutdown})
{
}

std::string PickState::execute(yasmin::Blackboard::SharedPtr blackboard)
{
  runtime_->set_state(MissionStatus::STATE_RUNNING, "PICK", "");
  const auto slot = get_slot(blackboard, "pickup_slot");
  if (slot >= CompetitionModel::kSlotCount || !blackboard->contains("cargo")) {
    return outcomes::kFailed;
  }
  const auto cargo = blackboard->get<std::string>("cargo");
  const auto layer = runtime_->model().pickup_layer(slot);
  const auto result = runtime_->manipulate("pickup", "pick", slot, layer, cargo);
  if (result != ActionResult::kSucceeded) {
    return action_outcome(result);
  }
  return runtime_->model().confirm_pick(slot, cargo) ? outcomes::kOk : outcomes::kFailed;
}

NavParkState::NavParkState(Runtime::SharedPtr runtime)
: RuntimeState(
    std::move(runtime),
    {outcomes::kOk, outcomes::kFailed, outcomes::kReset, outcomes::kRecovery,
      outcomes::kShutdown})
{
}

std::string NavParkState::execute(yasmin::Blackboard::SharedPtr blackboard)
{
  runtime_->set_state(MissionStatus::STATE_RUNNING, "NAV_PARK", "");
  if (!blackboard->contains("destination")) {
    return outcomes::kFailed;
  }
  return action_outcome(runtime_->navigate(blackboard->get<std::string>("destination")));
}

ObserveParkState::ObserveParkState(Runtime::SharedPtr runtime)
: RuntimeState(
    std::move(runtime),
    {outcomes::kOk, outcomes::kFailed, outcomes::kReset, outcomes::kRecovery,
      outcomes::kShutdown})
{
}

std::string ObserveParkState::execute(yasmin::Blackboard::SharedPtr blackboard)
{
  runtime_->set_state(MissionStatus::STATE_RUNNING, "OBSERVE_PARK", "");
  if (!blackboard->contains("destination")) {
    return outcomes::kFailed;
  }
  const auto park = blackboard->get<std::string>("destination");
  const auto slot = get_slot(blackboard, "park_slot");
  if (slot >= CompetitionModel::kSlotCount) {
    return outcomes::kFailed;
  }
  const auto layer = runtime_->model().park_layer(park, slot);
  const auto observation = runtime_->observe_with_recovery(park, slot, layer);
  return action_outcome(observation.result);
}

PlaceState::PlaceState(Runtime::SharedPtr runtime)
: RuntimeState(
    std::move(runtime),
    {outcomes::kOk, outcomes::kFailed, outcomes::kReset, outcomes::kRecovery,
      outcomes::kShutdown})
{
}

std::string PlaceState::execute(yasmin::Blackboard::SharedPtr blackboard)
{
  runtime_->set_state(MissionStatus::STATE_RUNNING, "PLACE", "");
  if (!blackboard->contains("destination") || !blackboard->contains("cargo")) {
    return outcomes::kFailed;
  }
  const auto park = blackboard->get<std::string>("destination");
  const auto cargo = blackboard->get<std::string>("cargo");
  const auto slot = get_slot(blackboard, "park_slot");
  if (slot >= CompetitionModel::kSlotCount) {
    return outcomes::kFailed;
  }
  const auto layer = runtime_->model().park_layer(park, slot);
  const auto result = runtime_->manipulate(park, "place", slot, layer, cargo);
  if (result != ActionResult::kSucceeded) {
    return action_outcome(result);
  }
  return runtime_->model().confirm_place(park, slot) ? outcomes::kOk : outcomes::kFailed;
}

CheckDoneState::CheckDoneState(Runtime::SharedPtr runtime)
: RuntimeState(std::move(runtime), {outcomes::kNext, outcomes::kRouteDone})
{
}

std::string CheckDoneState::execute(yasmin::Blackboard::SharedPtr blackboard)
{
  (void)blackboard;
  runtime_->set_state(MissionStatus::STATE_RUNNING, "CHECK_DONE", "");
  return runtime_->model().done() ? outcomes::kRouteDone : outcomes::kNext;
}

ReportDoneState::ReportDoneState(Runtime::SharedPtr runtime)
: RuntimeState(std::move(runtime), {outcomes::kOk, outcomes::kRecovery})
{
}

std::string ReportDoneState::execute(yasmin::Blackboard::SharedPtr blackboard)
{
  (void)blackboard;
  runtime_->set_state(MissionStatus::STATE_REPORTING_DONE, "REPORT_DONE", "");
  runtime_->safe_stop("mission done");
  return runtime_->report_done() ? outcomes::kOk : outcomes::kRecovery;
}

ReportFailState::ReportFailState(Runtime::SharedPtr runtime)
: RuntimeState(std::move(runtime), {outcomes::kOk, outcomes::kRecovery})
{
}

std::string ReportFailState::execute(yasmin::Blackboard::SharedPtr blackboard)
{
  runtime_->set_state(MissionStatus::STATE_REPORTING_FAIL, "REPORT_FAIL", "");
  runtime_->safe_stop("mission failed");
  const auto code = blackboard->contains("error_code") ?
    static_cast<int16_t>(blackboard->get<int32_t>("error_code")) : static_cast<int16_t>(1);
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
  while (rclcpp::ok() && !is_canceled()) {
    const auto result = runtime_->wait_reset();
    if (result == WaitResult::kSuccess) {
      return outcomes::kOk;
    }
    if (result == WaitResult::kShutdown) {
      return outcomes::kShutdown;
    }
  }
  return outcomes::kShutdown;
}

}  // namespace atlas_mission_yasmin
