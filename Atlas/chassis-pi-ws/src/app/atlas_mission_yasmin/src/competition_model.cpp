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

#include "atlas_mission_yasmin/competition_model.hpp"

#include <algorithm>
#include <limits>
#include <numeric>

namespace atlas_mission_yasmin
{

namespace
{
constexpr std::size_t kInvalidSlot = std::numeric_limits<std::size_t>::max();
const std::string kEmptyDestination;
}

CompetitionModel::CompetitionModel()
{
  reset();
}

void CompetitionModel::reset()
{
  arena_.clear();
  park_1_cargo_.clear();
  park_2_cargo_.clear();
  held_cargo_.clear();
  pickup_layers_.fill(kInitialPickupLayers);
  park_1_layers_.fill(0);
  park_2_layers_.fill(0);
  delivered_total_ = 0;

  pickup_phase_ = PickupPhase::kPrimaryFirstHalf;
  pickup_round_ = 1;
  pickup_slot_ = 0;
  retry_begin_ = 0;
  retry_end_ = 0;
  retry_pass_progress_ = false;
}

bool CompetitionModel::valid_cargo(const std::string & cargo)
{
  return cargo == "gear" || cargo == "t_bolt";
}

bool CompetitionModel::valid_park(const std::string & park)
{
  return park == "park_1" || park == "park_2";
}

std::size_t CompetitionModel::slot_for_round(const uint8_t round)
{
  if (round < 1 || round > 8) {
    return kInvalidSlot;
  }
  return static_cast<std::size_t>((round - 1U) / 2U);
}

uint8_t CompetitionModel::round_for_slot_layer(const std::size_t slot, const uint8_t layer)
{
  if (slot >= kSlotCount || (layer != kHighPickupLayer && layer != kLowPickupLayer)) {
    return 0;
  }
  return static_cast<uint8_t>(slot * 2U + (layer == kHighPickupLayer ? 1U : 2U));
}

bool CompetitionModel::set_sorting_rule(
  const std::string & arena,
  const std::string & park_1_cargo,
  const std::string & park_2_cargo)
{
  if ((arena != "A" && arena != "B") ||
    !valid_cargo(park_1_cargo) ||
    !valid_cargo(park_2_cargo) ||
    park_1_cargo == park_2_cargo)
  {
    return false;
  }

  // One observation establishes the rule for the entire run. Repeated identical
  // observations are harmless, but a conflicting result must not reroute cargo.
  if (!arena_.empty()) {
    return arena_ == arena && park_1_cargo_ == park_1_cargo &&
           park_2_cargo_ == park_2_cargo;
  }

  arena_ = arena;
  park_1_cargo_ = park_1_cargo;
  park_2_cargo_ = park_2_cargo;
  return true;
}

const std::string & CompetitionModel::arena() const
{
  return arena_;
}

const std::string & CompetitionModel::destination_for(const std::string & cargo) const
{
  if (cargo == park_1_cargo_) {
    static const std::string park_1 = "park_1";
    return park_1;
  }
  if (cargo == park_2_cargo_) {
    static const std::string park_2 = "park_2";
    return park_2;
  }
  return kEmptyDestination;
}

bool CompetitionModel::pickup_phase_is_retry() const
{
  return pickup_phase_ == PickupPhase::kRetryFirstHalf ||
         pickup_phase_ == PickupPhase::kRetrySecondHalf;
}

bool CompetitionModel::pickup_group_has_remaining(
  const std::size_t begin, const std::size_t end) const
{
  if (begin >= end || end > kSlotCount) {
    return false;
  }
  for (std::size_t slot = begin; slot < end; ++slot) {
    if (pickup_layers_[slot] > 0) {
      return true;
    }
  }
  return false;
}

std::size_t CompetitionModel::first_remaining_slot(
  const std::size_t begin, const std::size_t end) const
{
  if (begin >= end || end > kSlotCount) {
    return kInvalidSlot;
  }
  for (std::size_t slot = begin; slot < end; ++slot) {
    if (pickup_layers_[slot] > 0) {
      return slot;
    }
  }
  return kInvalidSlot;
}

std::size_t CompetitionModel::next_remaining_slot(
  const std::size_t after, const std::size_t begin, const std::size_t end) const
{
  if (begin >= end || end > kSlotCount) {
    return kInvalidSlot;
  }
  const std::size_t start = std::max(begin, after + 1U);
  for (std::size_t slot = start; slot < end; ++slot) {
    if (pickup_layers_[slot] > 0) {
      return slot;
    }
  }
  return kInvalidSlot;
}

std::size_t CompetitionModel::next_pickup_slot() const
{
  if (pickup_phase_ == PickupPhase::kComplete || pickup_phase_ == PickupPhase::kStalled ||
    pickup_slot_ >= kSlotCount || pickup_layers_[pickup_slot_] == 0)
  {
    return kInvalidSlot;
  }
  return pickup_slot_;
}

uint8_t CompetitionModel::pickup_layer(const std::size_t slot) const
{
  return slot < pickup_layers_.size() ? pickup_layers_[slot] : 0;
}

uint8_t CompetitionModel::pickup_round() const
{
  return pickup_round_;
}

bool CompetitionModel::pickup_retry_phase() const
{
  return pickup_phase_is_retry();
}

bool CompetitionModel::pickup_stalled() const
{
  return pickup_phase_ == PickupPhase::kStalled;
}

void CompetitionModel::begin_second_half()
{
  pickup_phase_ = PickupPhase::kPrimarySecondHalf;
  pickup_round_ = 5;
  pickup_slot_ = slot_for_round(pickup_round_);
  retry_begin_ = 0;
  retry_end_ = 0;
  retry_pass_progress_ = false;
}

void CompetitionModel::finish_pickup_schedule()
{
  pickup_phase_ = PickupPhase::kComplete;
  pickup_round_ = 0;
  pickup_slot_ = kInvalidSlot;
  retry_begin_ = 0;
  retry_end_ = 0;
  retry_pass_progress_ = false;
}

void CompetitionModel::enter_retry_phase(
  const std::size_t begin, const std::size_t end, const PickupPhase phase)
{
  retry_begin_ = begin;
  retry_end_ = end;
  retry_pass_progress_ = false;
  pickup_phase_ = phase;
  pickup_slot_ = first_remaining_slot(begin, end);
  if (pickup_slot_ >= kSlotCount) {
    if (phase == PickupPhase::kRetryFirstHalf) {
      begin_second_half();
    } else {
      finish_pickup_schedule();
    }
    return;
  }
  pickup_round_ = round_for_slot_layer(pickup_slot_, pickup_layers_[pickup_slot_]);
}

void CompetitionModel::advance_primary_after_attempt(
  const bool success, const uint8_t attempted_layer)
{
  // Odd rounds are the normal high-layer pass. Whether they succeed or fail,
  // the following even-round stage remains on the same physical slot.
  if ((pickup_round_ % 2U) == 1U) {
    ++pickup_round_;
    pickup_slot_ = slot_for_round(pickup_round_);
    return;
  }

  // If an odd-round high pick failed, the even round retries that high layer.
  // On success, keep the same even-round stage once more so it can transport
  // the low layer from the same point before moving on.
  if (success && attempted_layer == kHighPickupLayer &&
    pickup_slot_ < kSlotCount && pickup_layers_[pickup_slot_] == kLowPickupLayer)
  {
    return;
  }

  // Normal even-round low attempt, or an even-round high retry that failed:
  // advance to the next physical point. At the 1..4 / 5..8 barriers, first
  // clear deferred failures in the current half to avoid arm/cargo collision.
  if (pickup_round_ == 4U) {
    if (pickup_group_has_remaining(0, 2)) {
      enter_retry_phase(0, 2, PickupPhase::kRetryFirstHalf);
    } else {
      begin_second_half();
    }
    return;
  }

  if (pickup_round_ == 8U) {
    if (pickup_group_has_remaining(2, 4)) {
      enter_retry_phase(2, 4, PickupPhase::kRetrySecondHalf);
    } else {
      finish_pickup_schedule();
    }
    return;
  }

  ++pickup_round_;
  pickup_slot_ = slot_for_round(pickup_round_);
}

void CompetitionModel::finish_retry_pass()
{
  if (!pickup_group_has_remaining(retry_begin_, retry_end_)) {
    if (pickup_phase_ == PickupPhase::kRetryFirstHalf) {
      begin_second_half();
    } else {
      finish_pickup_schedule();
    }
    return;
  }

  if (!retry_pass_progress_) {
    // A complete deferred-retry pass made zero progress. Do not continue into
    // the next half with cargo still standing in a collision-sensitive area.
    pickup_phase_ = PickupPhase::kStalled;
    pickup_round_ = 0;
    pickup_slot_ = kInvalidSlot;
    return;
  }

  retry_pass_progress_ = false;
  pickup_slot_ = first_remaining_slot(retry_begin_, retry_end_);
  if (pickup_slot_ >= kSlotCount) {
    if (pickup_phase_ == PickupPhase::kRetryFirstHalf) {
      begin_second_half();
    } else {
      finish_pickup_schedule();
    }
    return;
  }
  pickup_round_ = round_for_slot_layer(pickup_slot_, pickup_layers_[pickup_slot_]);
}

void CompetitionModel::advance_retry_after_attempt(const bool success)
{
  if (success) {
    retry_pass_progress_ = true;
  }

  // After a successful high-layer retry, immediately stay at this physical
  // point and transport its newly exposed low layer before moving away.
  if (success && pickup_slot_ < kSlotCount && pickup_layers_[pickup_slot_] > 0) {
    pickup_round_ = round_for_slot_layer(pickup_slot_, pickup_layers_[pickup_slot_]);
    return;
  }

  const auto next = next_remaining_slot(pickup_slot_, retry_begin_, retry_end_);
  if (next < kSlotCount) {
    pickup_slot_ = next;
    pickup_round_ = round_for_slot_layer(pickup_slot_, pickup_layers_[pickup_slot_]);
    return;
  }

  finish_retry_pass();
}

bool CompetitionModel::record_pick_failure(const std::size_t slot)
{
  if (slot >= pickup_layers_.size() || slot != pickup_slot_ || pickup_layers_[slot] == 0 ||
    pickup_phase_ == PickupPhase::kComplete || pickup_phase_ == PickupPhase::kStalled)
  {
    return false;
  }

  const auto attempted_layer = pickup_layers_[slot];
  if (pickup_phase_is_retry()) {
    advance_retry_after_attempt(false);
  } else {
    advance_primary_after_attempt(false, attempted_layer);
  }
  return true;
}

bool CompetitionModel::confirm_pick(const std::size_t slot, const std::string & cargo)
{
  if (!held_cargo_.empty() || slot >= pickup_layers_.size() || slot != pickup_slot_ ||
    pickup_layers_[slot] == 0 || destination_for(cargo).empty() ||
    pickup_phase_ == PickupPhase::kComplete || pickup_phase_ == PickupPhase::kStalled)
  {
    return false;
  }

  const auto attempted_layer = pickup_layers_[slot];
  --pickup_layers_[slot];
  held_cargo_ = cargo;

  if (pickup_phase_is_retry()) {
    advance_retry_after_attempt(true);
  } else {
    advance_primary_after_attempt(true, attempted_layer);
  }
  return true;
}

std::size_t CompetitionModel::next_park_slot(const std::string & park) const
{
  if (!valid_park(park)) {
    return kInvalidSlot;
  }
  const auto & layers = park == "park_1" ? park_1_layers_ : park_2_layers_;
  const auto count = std::accumulate(layers.begin(), layers.end(), 0U);
  if (count >= kCargoTotal / 2) {
    return kInvalidSlot;
  }
  return static_cast<std::size_t>(
    std::distance(layers.begin(), std::min_element(layers.begin(), layers.end())));
}

uint8_t CompetitionModel::park_layer(const std::string & park, const std::size_t slot) const
{
  if (!valid_park(park) || slot >= kSlotCount) {
    return 0;
  }
  const auto & layers = park == "park_1" ? park_1_layers_ : park_2_layers_;
  return layers[slot];
}

bool CompetitionModel::can_place(const std::string & park, const std::string & cargo) const
{
  return valid_park(park) && !held_cargo_.empty() &&
         held_cargo_ == cargo && destination_for(cargo) == park;
}

bool CompetitionModel::confirm_place(
  const std::string & park, const std::size_t slot, const std::string & cargo)
{
  if (!can_place(park, cargo) || slot >= kSlotCount || delivered_total_ >= kCargoTotal)
  {
    return false;
  }
  auto & layers = park == "park_1" ? park_1_layers_ : park_2_layers_;
  if (std::accumulate(layers.begin(), layers.end(), 0U) >= kCargoTotal / 2) {
    return false;
  }
  ++layers[slot];
  ++delivered_total_;
  held_cargo_.clear();
  return true;
}

uint8_t CompetitionModel::delivered_total() const
{
  return delivered_total_;
}

bool CompetitionModel::done() const
{
  return delivered_total_ >= kCargoTotal && pickup_phase_ == PickupPhase::kComplete;
}

}  // namespace atlas_mission_yasmin
