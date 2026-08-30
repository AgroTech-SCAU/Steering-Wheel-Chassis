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

#include <gtest/gtest.h>

#include <array>
#include <string>

#include "atlas_mission_yasmin/competition_model.hpp"

using atlas_mission_yasmin::CompetitionModel;

TEST(CompetitionModelTest, CompletesEightCargoWithMinimalLayerContext)
{
  CompetitionModel model;
  ASSERT_TRUE(model.set_sorting_rule("B", "t_bolt", "gear"));
  EXPECT_EQ(model.destination_for("gear"), "park_2");
  EXPECT_EQ(model.destination_for("t_bolt"), "park_1");

  const std::array<std::string, 8> cargo = {
    "gear", "t_bolt", "gear", "t_bolt",
    "gear", "t_bolt", "gear", "t_bolt",
  };

  for (const auto & item : cargo) {
    const auto pickup_slot = model.next_pickup_slot();
    ASSERT_LT(pickup_slot, CompetitionModel::kSlotCount);
    ASSERT_GT(model.pickup_layer(pickup_slot), 0U);
    ASSERT_TRUE(model.confirm_pick(pickup_slot, item));

    const auto park = model.destination_for(item);
    const auto park_slot = model.next_park_slot(park);
    ASSERT_LT(park_slot, CompetitionModel::kSlotCount);
    ASSERT_TRUE(model.confirm_place(park, park_slot));
  }

  EXPECT_TRUE(model.done());
  EXPECT_EQ(model.delivered_total(), CompetitionModel::kCargoTotal);
  for (std::size_t slot = 0; slot < CompetitionModel::kSlotCount; ++slot) {
    EXPECT_EQ(model.pickup_layer(slot), 0U);
  }
}

TEST(CompetitionModelTest, RejectsInvalidSortingRule)
{
  CompetitionModel model;
  EXPECT_FALSE(model.set_sorting_rule("C", "gear", "t_bolt"));
  EXPECT_FALSE(model.set_sorting_rule("A", "gear", "gear"));
}
