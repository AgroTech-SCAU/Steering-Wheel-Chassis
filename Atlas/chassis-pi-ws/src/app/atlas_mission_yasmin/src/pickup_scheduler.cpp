// Copyright 2026 yangxuan
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "atlas_mission_yasmin/pickup_scheduler.hpp"

#include <algorithm>

namespace atlas_mission_yasmin
{

PickupScheduler::PickupScheduler()
{
  reset();
}

void PickupScheduler::reset()
{
  mode_ = Mode::kPrimary;
  group_begin_ = 0;
  group_end_ = 2;
  current_slot_ = 0;
  primary_visit_ = 0;
  retry_pass_progress_ = false;
  retry_followup_low_ = false;
  abandoned_total_ = 0;
}

bool PickupScheduler::group_has_remaining(const RemainingLayers & remaining) const
{
  for (std::size_t slot = group_begin_; slot < group_end_; ++slot) {
    if (remaining[slot] > 0) {
      return true;
    }
  }
  return false;
}

std::size_t PickupScheduler::first_remaining_slot(const RemainingLayers & remaining) const
{
  for (std::size_t slot = group_begin_; slot < group_end_; ++slot) {
    if (remaining[slot] > 0) {
      return slot;
    }
  }
  return kInvalidSlot;
}

std::size_t PickupScheduler::next_remaining_slot(
  const RemainingLayers & remaining, const std::size_t after) const
{
  for (std::size_t slot = std::max(group_begin_, after + 1U); slot < group_end_; ++slot) {
    if (remaining[slot] > 0) {
      return slot;
    }
  }
  return kInvalidSlot;
}

uint8_t PickupScheduler::remaining_count(const RemainingLayers & remaining) const
{
  uint8_t count = 0;
  for (std::size_t slot = group_begin_; slot < group_end_; ++slot) {
    count = static_cast<uint8_t>(count + remaining[slot]);
  }
  return count;
}

std::size_t PickupScheduler::next_slot(const RemainingLayers & remaining) const
{
  if (mode_ == Mode::kComplete || current_slot_ >= kSlotCount || remaining[current_slot_] == 0) {
    return kInvalidSlot;
  }
  return current_slot_;
}

uint8_t PickupScheduler::nominal_round(const RemainingLayers & remaining) const
{
  if (mode_ == Mode::kComplete || current_slot_ >= kSlotCount) {
    return 0;
  }

  // `round` is display/reporting metadata only. Scheduling never branches on it.
  if (mode_ == Mode::kPrimary) {
    return static_cast<uint8_t>(current_slot_ * 2U + (primary_visit_ == 0 ? 1U : 2U));
  }

  const auto layer = remaining[current_slot_];
  if (layer == kHighLayer) {
    return static_cast<uint8_t>(current_slot_ * 2U + 1U);
  }
  if (layer == kLowLayer) {
    return static_cast<uint8_t>(current_slot_ * 2U + 2U);
  }
  return 0;
}

bool PickupScheduler::retry_phase() const
{
  return mode_ == Mode::kRetry;
}

bool PickupScheduler::complete() const
{
  return mode_ == Mode::kComplete;
}

uint8_t PickupScheduler::abandoned_total() const
{
  return abandoned_total_;
}

void PickupScheduler::advance_group_or_complete()
{
  retry_pass_progress_ = false;
  retry_followup_low_ = false;
  primary_visit_ = 0;

  if (group_begin_ == 0) {
    group_begin_ = 2;
    group_end_ = 4;
    current_slot_ = 2;
    mode_ = Mode::kPrimary;
    return;
  }

  mode_ = Mode::kComplete;
  current_slot_ = kInvalidSlot;
}

void PickupScheduler::enter_retry(const RemainingLayers & remaining)
{
  mode_ = Mode::kRetry;
  retry_pass_progress_ = false;
  retry_followup_low_ = false;
  current_slot_ = first_remaining_slot(remaining);
  if (current_slot_ == kInvalidSlot) {
    advance_group_or_complete();
  }
}

void PickupScheduler::finish_primary_group(const RemainingLayers & remaining)
{
  if (group_has_remaining(remaining)) {
    enter_retry(remaining);
  } else {
    advance_group_or_complete();
  }
}

void PickupScheduler::advance_primary_slot(const RemainingLayers & remaining)
{
  primary_visit_ = 0;
  if (current_slot_ + 1U < group_end_) {
    ++current_slot_;
    return;
  }
  finish_primary_group(remaining);
}

void PickupScheduler::finish_retry_pass(const RemainingLayers & remaining)
{
  if (!group_has_remaining(remaining)) {
    advance_group_or_complete();
    return;
  }

  if (!retry_pass_progress_) {
    // Competition fail-soft policy: one complete retry pass produced no pickup.
    // Keep physical remaining layers untouched, mark them abandoned for metrics,
    // and continue the full route instead of trapping AUTO in this half.
    abandoned_total_ = static_cast<uint8_t>(abandoned_total_ + remaining_count(remaining));
    advance_group_or_complete();
    return;
  }

  retry_pass_progress_ = false;
  retry_followup_low_ = false;
  current_slot_ = first_remaining_slot(remaining);
  if (current_slot_ == kInvalidSlot) {
    advance_group_or_complete();
  }
}

void PickupScheduler::advance_retry_slot(const RemainingLayers & remaining)
{
  const auto next = next_remaining_slot(remaining, current_slot_);
  if (next != kInvalidSlot) {
    current_slot_ = next;
    return;
  }
  finish_retry_pass(remaining);
}

bool PickupScheduler::record_attempt(
  const bool success,
  const uint8_t attempted_layer,
  const RemainingLayers & remaining_after)
{
  if (mode_ == Mode::kComplete || current_slot_ >= kSlotCount ||
    (attempted_layer != kHighLayer && attempted_layer != kLowLayer))
  {
    return false;
  }

  if (mode_ == Mode::kPrimary) {
    // Follow-up after the second nominal visit: one low-layer attempt, then leave
    // this physical point regardless of outcome. Any remaining cargo is deferred.
    if (primary_visit_ >= 2U) {
      advance_primary_slot(remaining_after);
      return true;
    }

    if (success && remaining_after[current_slot_] == 0) {
      advance_primary_slot(remaining_after);
      return true;
    }

    if (primary_visit_ == 0U) {
      // First nominal visit always leads to the second visit at the same point.
      primary_visit_ = 1U;
      return true;
    }

    // Second nominal visit finally removed the high layer: immediately service
    // the newly exposed low layer once before moving to the next point.
    if (success && attempted_layer == kHighLayer &&
      remaining_after[current_slot_] == kLowLayer)
    {
      primary_visit_ = 2U;
      return true;
    }

    advance_primary_slot(remaining_after);
    return true;
  }

  if (success) {
    retry_pass_progress_ = true;
  }

  if (retry_followup_low_) {
    retry_followup_low_ = false;
    advance_retry_slot(remaining_after);
    return true;
  }

  // A successful high-layer retry exposes the low layer at the same point.
  // Clear that low layer once before advancing the retry cursor.
  if (success && attempted_layer == kHighLayer &&
    remaining_after[current_slot_] == kLowLayer)
  {
    retry_followup_low_ = true;
    return true;
  }

  advance_retry_slot(remaining_after);
  return true;
}

}  // namespace atlas_mission_yasmin
