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

  CompetitionModel();

  void reset();
  bool set_sorting_rule(
    const std::string & arena,
    const std::string & park_1_cargo,
    const std::string & park_2_cargo);

  const std::string & arena() const;
  const std::string & destination_for(const std::string & cargo) const;

  std::size_t next_pickup_slot() const;
  uint8_t pickup_layer(std::size_t slot) const;
  bool confirm_pick(std::size_t slot, const std::string & cargo);

  std::size_t next_park_slot(const std::string & park) const;
  uint8_t park_layer(const std::string & park, std::size_t slot) const;
  bool confirm_place(const std::string & park, std::size_t slot);

  uint8_t delivered_total() const;
  bool done() const;

private:
  static bool valid_cargo(const std::string & cargo);
  static bool valid_park(const std::string & park);

  std::string arena_;
  std::string park_1_cargo_;
  std::string park_2_cargo_;
  std::array<uint8_t, kSlotCount> pickup_layers_{};
  std::array<uint8_t, kSlotCount> park_1_layers_{};
  std::array<uint8_t, kSlotCount> park_2_layers_{};
  uint8_t delivered_total_{0};
};

}  // namespace atlas_mission_yasmin

#endif  // ATLAS_MISSION_YASMIN__COMPETITION_MODEL_HPP_
