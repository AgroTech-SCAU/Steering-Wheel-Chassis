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

#ifndef ATLAS_MISSION_YASMIN__COMPETITION_MODEL_HPP_
#define ATLAS_MISSION_YASMIN__COMPETITION_MODEL_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace atlas_mission_yasmin
{

class CompetitionModel
{
public:
  static constexpr std::size_t kSlotCount = 4;
  static constexpr uint8_t kInitialPickupLayers = 2;
  static constexpr uint8_t kCargoTotal = 8;
  static constexpr uint8_t kHighPickupLayer = 2;
  static constexpr uint8_t kLowPickupLayer = 1;

  CompetitionModel();

  void reset();
  bool set_sorting_rule(
    const std::string & arena,
    const std::string & park_1_cargo,
    const std::string & park_2_cargo);

  const std::string & arena() const;
  const std::string & destination_for(const std::string & cargo) const;

  // Pickup scheduling semantics:
  // - rounds 1/2 -> slot 0, rounds 3/4 -> slot 1,
  //   rounds 5/6 -> slot 2, rounds 7/8 -> slot 3.
  // - the physical top remaining layer is authoritative: 2=high, 1=low.
  //   Therefore an odd-round high-layer miss makes the following even round
  //   retry the high layer; after that succeeds the same even-round stage is
  //   used once more for the low layer.
  // - after rounds 1..4, unresolved cargo in slots 0..1 is retried before
  //   rounds 5..8. Slots 2..3 use the same deferred-retry rule at the end.
  std::size_t next_pickup_slot() const;
  uint8_t pickup_layer(std::size_t slot) const;
  uint8_t pickup_round() const;
  bool pickup_retry_phase() const;
  bool pickup_stalled() const;
  bool pickup_schedule_complete() const;
  bool record_pick_failure(std::size_t slot);
  bool confirm_pick(std::size_t slot, const std::string & cargo);

  std::size_t next_park_slot(const std::string & park) const;
  uint8_t park_layer(const std::string & park, std::size_t slot) const;
  bool can_place(const std::string & park, const std::string & cargo) const;
  bool confirm_place(const std::string & park, std::size_t slot, const std::string & cargo);

  uint8_t delivered_total() const;
  bool done() const;

private:
  enum class PickupPhase : uint8_t
  {
    kPrimaryFirstHalf,
    kRetryFirstHalf,
    kPrimarySecondHalf,
    kRetrySecondHalf,
    kComplete,
    kStalled,
  };

  static bool valid_cargo(const std::string & cargo);
  static bool valid_park(const std::string & park);
  static std::size_t slot_for_round(uint8_t round);
  static uint8_t round_for_slot_layer(std::size_t slot, uint8_t layer);

  bool pickup_phase_is_retry() const;
  bool pickup_group_has_remaining(std::size_t begin, std::size_t end) const;
  std::size_t first_remaining_slot(std::size_t begin, std::size_t end) const;
  std::size_t next_remaining_slot(std::size_t after, std::size_t begin, std::size_t end) const;
  void advance_primary_after_attempt(bool success, uint8_t attempted_layer);
  void enter_retry_phase(std::size_t begin, std::size_t end, PickupPhase phase);
  void advance_retry_after_attempt(bool success);
  void finish_retry_pass();
  void begin_second_half();
  void finish_pickup_schedule();

  std::string arena_;
  std::string park_1_cargo_;
  std::string park_2_cargo_;
  std::string held_cargo_;
  std::array<uint8_t, kSlotCount> pickup_layers_{};
  std::array<uint8_t, kSlotCount> park_1_layers_{};
  std::array<uint8_t, kSlotCount> park_2_layers_{};
  uint8_t delivered_total_{0};

  PickupPhase pickup_phase_{PickupPhase::kPrimaryFirstHalf};
  uint8_t pickup_round_{1};
  std::size_t pickup_slot_{0};
  std::size_t retry_begin_{0};
  std::size_t retry_end_{0};
  bool retry_pass_progress_{false};
};

}  // namespace atlas_mission_yasmin

#endif  // ATLAS_MISSION_YASMIN__COMPETITION_MODEL_HPP_
