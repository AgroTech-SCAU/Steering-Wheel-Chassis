// Copyright 2026 yangxuan
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef ATLAS_MISSION_YASMIN__PICKUP_SCHEDULER_HPP_
#define ATLAS_MISSION_YASMIN__PICKUP_SCHEDULER_HPP_

#include <array>
#include <cstddef>
#include <cstdint>

namespace atlas_mission_yasmin
{

class PickupScheduler
{
public:
  static constexpr std::size_t kSlotCount = 4;
  static constexpr uint8_t kHighLayer = 2;
  static constexpr uint8_t kLowLayer = 1;
  using RemainingLayers = std::array<uint8_t, kSlotCount>;

  PickupScheduler();

  void reset();

  std::size_t next_slot(const RemainingLayers & remaining) const;
  uint8_t nominal_round(const RemainingLayers & remaining) const;
  bool retry_phase() const;
  bool complete() const;
  uint8_t abandoned_total() const;

  // Record one physical pickup attempt. `remaining_after` must reflect the
  // physical scene after the attempt: unchanged on miss/failure, decremented
  // by the model on a successful pick.
  bool record_attempt(
    bool success,
    uint8_t attempted_layer,
    const RemainingLayers & remaining_after);

private:
  enum class Mode : uint8_t
  {
    kPrimary,
    kRetry,
    kComplete,
  };

  static constexpr std::size_t kInvalidSlot = static_cast<std::size_t>(-1);

  bool group_has_remaining(const RemainingLayers & remaining) const;
  std::size_t first_remaining_slot(const RemainingLayers & remaining) const;
  std::size_t next_remaining_slot(
    const RemainingLayers & remaining,
    std::size_t after) const;
  uint8_t remaining_count(const RemainingLayers & remaining) const;

  void advance_primary_slot(const RemainingLayers & remaining);
  void finish_primary_group(const RemainingLayers & remaining);
  void enter_retry(const RemainingLayers & remaining);
  void advance_retry_slot(const RemainingLayers & remaining);
  void finish_retry_pass(const RemainingLayers & remaining);
  void advance_group_or_complete();

  Mode mode_{Mode::kPrimary};
  std::size_t group_begin_{0};
  std::size_t group_end_{2};
  std::size_t current_slot_{0};

  // Primary visit 0/1 are the two nominal visits for one pickup point.
  // Value 2 is a one-shot low-layer follow-up when the second nominal visit
  // finally removed the high layer.
  uint8_t primary_visit_{0};

  bool retry_pass_progress_{false};
  bool retry_followup_low_{false};
  uint8_t abandoned_total_{0};
};

}  // namespace atlas_mission_yasmin

#endif  // ATLAS_MISSION_YASMIN__PICKUP_SCHEDULER_HPP_
