// Copyright 2026 yangxuan
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <gtest/gtest.h>

#include <array>
#include <string>

#include "atlas_mission_yasmin/competition_model.hpp"

using atlas_mission_yasmin::CompetitionModel;

namespace
{

void confirm_current_delivery(CompetitionModel & model, const std::string & cargo)
{
  const auto slot = model.next_pickup_slot();
  ASSERT_LT(slot, CompetitionModel::kSlotCount);
  ASSERT_GT(model.pickup_layer(slot), 0U);
  ASSERT_TRUE(model.confirm_pick(slot, cargo));

  const auto park = model.destination_for(cargo);
  const auto park_slot = model.next_park_slot(park);
  ASSERT_LT(park_slot, CompetitionModel::kSlotCount);
  ASSERT_TRUE(model.confirm_place(park, park_slot, cargo));
}

}  // namespace

TEST(CompetitionModelTest, NormalFlowUsesPairedPointsAndHighThenLowLayers)
{
  CompetitionModel model;
  ASSERT_TRUE(model.set_sorting_rule("B", "t_bolt", "gear"));

  const std::array<std::size_t, 8> expected_slots = {0, 0, 1, 1, 2, 2, 3, 3};
  const std::array<uint8_t, 8> expected_layers = {2, 1, 2, 1, 2, 1, 2, 1};
  const std::array<uint8_t, 8> expected_rounds = {1, 2, 3, 4, 5, 6, 7, 8};
  const std::array<std::string, 8> cargo = {
    "gear", "t_bolt", "gear", "t_bolt",
    "gear", "t_bolt", "gear", "t_bolt",
  };

  for (std::size_t i = 0; i < cargo.size(); ++i) {
    EXPECT_EQ(model.next_pickup_slot(), expected_slots[i]);
    EXPECT_EQ(model.pickup_layer(expected_slots[i]), expected_layers[i]);
    EXPECT_EQ(model.pickup_round(), expected_rounds[i]);
    EXPECT_FALSE(model.pickup_retry_phase());
    confirm_current_delivery(model, cargo[i]);
  }

  EXPECT_TRUE(model.done());
  EXPECT_EQ(model.delivered_total(), CompetitionModel::kCargoTotal);
}

TEST(CompetitionModelTest, OddRoundMissMakesEvenRoundRetryHighThenRepeatEvenForLow)
{
  CompetitionModel model;
  ASSERT_TRUE(model.set_sorting_rule("A", "gear", "t_bolt"));

  ASSERT_EQ(model.pickup_round(), 1U);
  ASSERT_EQ(model.next_pickup_slot(), 0U);
  ASSERT_EQ(model.pickup_layer(0), CompetitionModel::kHighPickupLayer);
  ASSERT_TRUE(model.record_pick_failure(0));

  // Round 2 is now a retry of the still-present high layer.
  EXPECT_EQ(model.pickup_round(), 2U);
  EXPECT_EQ(model.next_pickup_slot(), 0U);
  EXPECT_EQ(model.pickup_layer(0), CompetitionModel::kHighPickupLayer);
  confirm_current_delivery(model, "gear");

  // High retry succeeded: stay in the round-2 stage for the low layer.
  EXPECT_EQ(model.pickup_round(), 2U);
  EXPECT_EQ(model.next_pickup_slot(), 0U);
  EXPECT_EQ(model.pickup_layer(0), CompetitionModel::kLowPickupLayer);
  confirm_current_delivery(model, "t_bolt");

  EXPECT_EQ(model.pickup_round(), 3U);
  EXPECT_EQ(model.next_pickup_slot(), 1U);
  EXPECT_EQ(model.pickup_layer(1), CompetitionModel::kHighPickupLayer);
}

TEST(CompetitionModelTest, TwoHighMissesAdvanceToNextPointAndRetryBeforeRoundFive)
{
  CompetitionModel model;
  ASSERT_TRUE(model.set_sorting_rule("A", "gear", "t_bolt"));

  ASSERT_TRUE(model.record_pick_failure(0));  // round 1 high miss
  ASSERT_EQ(model.pickup_round(), 2U);
  ASSERT_TRUE(model.record_pick_failure(0));  // round 2 high retry also misses

  EXPECT_EQ(model.pickup_round(), 3U);
  EXPECT_EQ(model.next_pickup_slot(), 1U);
  EXPECT_EQ(model.pickup_layer(1), CompetitionModel::kHighPickupLayer);

  confirm_current_delivery(model, "gear");   // round 3 high
  confirm_current_delivery(model, "t_bolt"); // round 4 low

  // Before entering rounds 5/6, slot 0 must be cleared in deferred retry.
  EXPECT_TRUE(model.pickup_retry_phase());
  EXPECT_EQ(model.next_pickup_slot(), 0U);
  EXPECT_EQ(model.pickup_round(), 1U);
  EXPECT_EQ(model.pickup_layer(0), CompetitionModel::kHighPickupLayer);

  confirm_current_delivery(model, "gear");
  EXPECT_TRUE(model.pickup_retry_phase());
  EXPECT_EQ(model.next_pickup_slot(), 0U);
  EXPECT_EQ(model.pickup_round(), 2U);
  EXPECT_EQ(model.pickup_layer(0), CompetitionModel::kLowPickupLayer);

  confirm_current_delivery(model, "t_bolt");
  EXPECT_FALSE(model.pickup_retry_phase());
  EXPECT_EQ(model.pickup_round(), 5U);
  EXPECT_EQ(model.next_pickup_slot(), 2U);
}

TEST(CompetitionModelTest, LowLayerMissIsDeferredUntilRoundsOneToFourFinish)
{
  CompetitionModel model;
  ASSERT_TRUE(model.set_sorting_rule("A", "gear", "t_bolt"));

  confirm_current_delivery(model, "gear");  // round 1 high succeeds
  ASSERT_EQ(model.pickup_round(), 2U);
  ASSERT_EQ(model.pickup_layer(0), CompetitionModel::kLowPickupLayer);
  ASSERT_TRUE(model.record_pick_failure(0)); // round 2 low misses

  EXPECT_EQ(model.pickup_round(), 3U);
  EXPECT_EQ(model.next_pickup_slot(), 1U);
  confirm_current_delivery(model, "gear");   // round 3 high
  confirm_current_delivery(model, "t_bolt"); // round 4 low

  EXPECT_TRUE(model.pickup_retry_phase());
  EXPECT_EQ(model.next_pickup_slot(), 0U);
  EXPECT_EQ(model.pickup_round(), 2U);
  EXPECT_EQ(model.pickup_layer(0), CompetitionModel::kLowPickupLayer);

  confirm_current_delivery(model, "t_bolt");
  EXPECT_EQ(model.pickup_round(), 5U);
  EXPECT_EQ(model.next_pickup_slot(), 2U);
}

TEST(CompetitionModelTest, RetryPassWithNoProgressAbandonsHalfAndContinuesRoute)
{
  CompetitionModel model;
  ASSERT_TRUE(model.set_sorting_rule("A", "gear", "t_bolt"));

  ASSERT_TRUE(model.record_pick_failure(0)); // r1
  ASSERT_TRUE(model.record_pick_failure(0)); // r2
  ASSERT_TRUE(model.record_pick_failure(1)); // r3
  ASSERT_TRUE(model.record_pick_failure(1)); // r4 -> retry first half

  ASSERT_TRUE(model.pickup_retry_phase());
  ASSERT_EQ(model.next_pickup_slot(), 0U);
  ASSERT_TRUE(model.record_pick_failure(0));
  ASSERT_EQ(model.next_pickup_slot(), 1U);
  ASSERT_TRUE(model.record_pick_failure(1));

  // Full-flow priority: first-half retry made no progress, but AUTO enters
  // rounds 5..8 instead of stalling the entire mission.
  EXPECT_FALSE(model.pickup_retry_phase());
  EXPECT_EQ(model.pickup_round(), 5U);
  EXPECT_EQ(model.next_pickup_slot(), 2U);
  EXPECT_FALSE(model.done());
  EXPECT_FALSE(model.pickup_schedule_complete());
  EXPECT_EQ(model.abandoned_total(), 4U);
}

TEST(CompetitionModelTest, FinalRetryNoProgressStillCompletesPickupSchedule)
{
  CompetitionModel model;
  ASSERT_TRUE(model.set_sorting_rule("A", "gear", "t_bolt"));

  // Abandon unresolved first half after its retry pass.
  for (std::size_t i = 0; i < 6; ++i) {
    const auto slot = model.next_pickup_slot();
    ASSERT_LT(slot, CompetitionModel::kSlotCount);
    ASSERT_TRUE(model.record_pick_failure(slot));
  }
  ASSERT_EQ(model.pickup_round(), 5U);

  // Fail rounds 5..8 and then the deferred retry of slots 2..3.
  for (std::size_t i = 0; i < 6; ++i) {
    const auto slot = model.next_pickup_slot();
    ASSERT_LT(slot, CompetitionModel::kSlotCount);
    ASSERT_TRUE(model.record_pick_failure(slot));
  }

  EXPECT_TRUE(model.pickup_schedule_complete());
  EXPECT_GE(model.next_pickup_slot(), CompetitionModel::kSlotCount);
  EXPECT_EQ(model.abandoned_total(), CompetitionModel::kCargoTotal);
  EXPECT_FALSE(model.done());  // no cargo delivered; schedule completion != perfect score
}

TEST(CompetitionModelTest, RejectsInvalidSortingRule)
{
  CompetitionModel model;
  EXPECT_FALSE(model.set_sorting_rule("C", "gear", "t_bolt"));
  EXPECT_FALSE(model.set_sorting_rule("A", "gear", "gear"));
}

TEST(CompetitionModelTest, SortingRuleCannotBeOverwrittenDuringRun)
{
  CompetitionModel model;
  ASSERT_TRUE(model.set_sorting_rule("A", "gear", "t_bolt"));
  EXPECT_TRUE(model.set_sorting_rule("A", "gear", "t_bolt"));
  EXPECT_FALSE(model.set_sorting_rule("B", "t_bolt", "gear"));
  EXPECT_FALSE(model.set_sorting_rule("A", "t_bolt", "gear"));
  EXPECT_EQ(model.arena(), "A");
  EXPECT_EQ(model.destination_for("gear"), "park_1");
  EXPECT_EQ(model.destination_for("t_bolt"), "park_2");

  model.reset();
  EXPECT_TRUE(model.set_sorting_rule("B", "t_bolt", "gear"));
  EXPECT_EQ(model.destination_for("gear"), "park_2");
}

TEST(CompetitionModelTest, CargoCannotBeConfirmedInWrongPark)
{
  CompetitionModel model;
  EXPECT_FALSE(model.confirm_pick(0, "gear"));
  EXPECT_FALSE(model.confirm_place("park_1", 0, "gear"));
  ASSERT_TRUE(model.set_sorting_rule("A", "gear", "t_bolt"));
  EXPECT_FALSE(model.confirm_place("park_2", 0, "gear"));
  EXPECT_EQ(model.delivered_total(), 0U);
  EXPECT_TRUE(model.confirm_pick(0, "gear"));
  EXPECT_FALSE(model.can_place("park_2", "gear"));
  EXPECT_FALSE(model.can_place("park_2", "t_bolt"));
  EXPECT_TRUE(model.can_place("park_1", "gear"));
  EXPECT_FALSE(model.confirm_pick(0, "t_bolt"));
  EXPECT_FALSE(model.confirm_place("park_1", 0, "t_bolt"));
  EXPECT_TRUE(model.confirm_place("park_1", 0, "gear"));
  EXPECT_EQ(model.delivered_total(), 1U);
}
