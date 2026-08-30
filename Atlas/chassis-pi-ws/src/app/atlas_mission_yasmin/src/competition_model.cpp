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
  pickup_layers_.fill(kInitialPickupLayers);
  park_1_layers_.fill(0);
  park_2_layers_.fill(0);
  delivered_total_ = 0;
}

bool CompetitionModel::valid_cargo(const std::string & cargo)
{
  return cargo == "gear" || cargo == "t_bolt";
}

bool CompetitionModel::valid_park(const std::string & park)
{
  return park == "park_1" || park == "park_2";
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

std::size_t CompetitionModel::next_pickup_slot() const
{
  for (std::size_t i = 0; i < pickup_layers_.size(); ++i) {
    if (pickup_layers_[i] > 0) {
      return i;
    }
  }
  return kInvalidSlot;
}

uint8_t CompetitionModel::pickup_layer(const std::size_t slot) const
{
  return slot < pickup_layers_.size() ? pickup_layers_[slot] : 0;
}

bool CompetitionModel::confirm_pick(const std::size_t slot, const std::string & cargo)
{
  if (slot >= pickup_layers_.size() || pickup_layers_[slot] == 0 || !valid_cargo(cargo)) {
    return false;
  }
  --pickup_layers_[slot];
  return true;
}

std::size_t CompetitionModel::next_park_slot(const std::string & park) const
{
  if (!valid_park(park)) {
    return kInvalidSlot;
  }
  const auto & layers = park == "park_1" ? park_1_layers_ : park_2_layers_;
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

bool CompetitionModel::confirm_place(const std::string & park, const std::size_t slot)
{
  if (!valid_park(park) || slot >= kSlotCount || delivered_total_ >= kCargoTotal) {
    return false;
  }
  auto & layers = park == "park_1" ? park_1_layers_ : park_2_layers_;
  ++layers[slot];
  ++delivered_total_;
  return true;
}

uint8_t CompetitionModel::delivered_total() const
{
  return delivered_total_;
}

bool CompetitionModel::done() const
{
  return delivered_total_ >= kCargoTotal;
}

}  // namespace atlas_mission_yasmin
