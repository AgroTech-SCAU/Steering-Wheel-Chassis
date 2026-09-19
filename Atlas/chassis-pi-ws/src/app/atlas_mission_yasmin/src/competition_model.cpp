// Copyright 2026 yangxuan
//
// Licensed under the Apache License, Version 2.0 (the "License");

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
  pickup_scheduler_.reset();
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

std::size_t CompetitionModel::next_pickup_slot() const
{
  return pickup_scheduler_.next_slot(pickup_layers_);
}

uint8_t CompetitionModel::pickup_layer(const std::size_t slot) const
{
  return slot < pickup_layers_.size() ? pickup_layers_[slot] : 0;
}

uint8_t CompetitionModel::pickup_round() const
{
  return pickup_scheduler_.nominal_round(pickup_layers_);
}

bool CompetitionModel::pickup_retry_phase() const
{
  return pickup_scheduler_.retry_phase();
}

bool CompetitionModel::pickup_schedule_complete() const
{
  return pickup_scheduler_.complete();
}

uint8_t CompetitionModel::abandoned_total() const
{
  return pickup_scheduler_.abandoned_total();
}

bool CompetitionModel::record_pick_failure(const std::size_t slot)
{
  if (slot >= pickup_layers_.size() || slot != next_pickup_slot() || pickup_layers_[slot] == 0) {
    return false;
  }
  const auto attempted_layer = pickup_layers_[slot];
  return pickup_scheduler_.record_attempt(false, attempted_layer, pickup_layers_);
}

bool CompetitionModel::confirm_pick(const std::size_t slot, const std::string & cargo)
{
  if (!held_cargo_.empty() || slot >= pickup_layers_.size() || slot != next_pickup_slot() ||
    pickup_layers_[slot] == 0 || destination_for(cargo).empty())
  {
    return false;
  }

  const auto attempted_layer = pickup_layers_[slot];
  --pickup_layers_[slot];
  held_cargo_ = cargo;
  if (!pickup_scheduler_.record_attempt(true, attempted_layer, pickup_layers_)) {
    ++pickup_layers_[slot];
    held_cargo_.clear();
    return false;
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
  if (!can_place(park, cargo) || slot >= kSlotCount || delivered_total_ >= kCargoTotal) {
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
  return delivered_total_ >= kCargoTotal && pickup_scheduler_.complete();
}

}  // namespace atlas_mission_yasmin
