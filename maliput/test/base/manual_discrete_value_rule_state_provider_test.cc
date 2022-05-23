// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2019-2022, Toyota Research Institute. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#include "maliput/base/manual_discrete_value_rule_state_provider.h"

#include <map>
#include <stdexcept>

#include <gtest/gtest.h>

#include "maliput/api/regions.h"
#include "maliput/api/rules/discrete_value_rule.h"
#include "maliput/api/rules/rule.h"
#include "maliput/base/manual_rulebook.h"
#include "maliput/base/rule_registry.h"
#include "maliput/base/rule_tools.h"
#include "maliput/common/assertion_error.h"
#include "maliput/test_utilities/mock.h"
#include "maliput/test_utilities/rules_compare.h"
#include "maliput/test_utilities/rules_test_utilities.h"

namespace maliput {
namespace test {
namespace {

using api::LaneId;
using api::LanePosition;
using api::LaneSRange;
using api::LaneSRoute;
using api::RoadPosition;
using api::rules::DiscreteValueRule;
using api::rules::DiscreteValueRuleStateProvider;
using api::rules::RoadRulebook;
using api::rules::Rule;

class ManualDiscreteRuleStateProviderTest : public ::testing::Test {
 protected:
  const Rule::Id kRuleId{"dvrt/dvr_id"};
  const Rule::TypeId kRuleType{"dvrt"};
  const Rule::Id kUnknownRuleId{"dvrt/unknown_id"};
  const LaneId kLaneId{"a"};
  const LaneSRange kLaneSRange{kLaneId, {0., 9.}};
  const DiscreteValueRule::DiscreteValue kStateA{DiscreteValueRule::DiscreteValue{
      Rule::State::kStrict, api::test::CreateEmptyRelatedRules(), api::test::CreateEmptyRelatedUniqueIds(), "value1"}};
  const DiscreteValueRule::DiscreteValue kStateB{DiscreteValueRule::DiscreteValue{
      Rule::State::kStrict, api::test::CreateEmptyRelatedRules(), api::test::CreateEmptyRelatedUniqueIds(), "value2"}};
  const DiscreteValueRule::DiscreteValue kInvalidState{
      DiscreteValueRule::DiscreteValue{Rule::State::kStrict, api::test::CreateEmptyRelatedRules(),
                                       api::test::CreateEmptyRelatedUniqueIds(), "invalid_state"}};
  const double kDurationUntil{10.};

  void SetUp() override {
    const maliput::api::test::RoadRulebookBuildFlags kRulebookBuildFlags{
        false /* add_right_of_way */, {} /* right_of_way_build_flags */,  false /* add_direction_usage */,
        false /* add_speed_limit */,  true /* add_discrete_value_rule */, false /* add_range_value_rule */};
    road_rulebook_ = api::test::CreateRoadRulebook(kRulebookBuildFlags);
  }

  std::unique_ptr<RoadRulebook> road_rulebook_;
};

TEST_F(ManualDiscreteRuleStateProviderTest, ConstructorConstraints) {
  EXPECT_THROW(ManualDiscreteValueRuleStateProvider(nullptr), maliput::common::assertion_error);
  EXPECT_NO_THROW(ManualDiscreteValueRuleStateProvider(road_rulebook_.get()));
}

TEST_F(ManualDiscreteRuleStateProviderTest, SetStateTest) {
  ManualDiscreteValueRuleStateProvider dut(road_rulebook_.get());

  // Tries to set the state to an unknown Rule::Id in `rulebook_`.
  EXPECT_THROW(dut.SetState(kUnknownRuleId, kStateA, {}, {}), std::out_of_range);
  // Tries to set an invalid state to the rule.
  EXPECT_THROW(dut.SetState(kRuleId, kInvalidState, {}, {}), maliput::common::assertion_error);
  // Tries to set an invalid next state to the rule.
  EXPECT_THROW(dut.SetState(kRuleId, kStateA, {kInvalidState}, {}), maliput::common::assertion_error);
  // Tries to set a valid next state with a negative duration.
  EXPECT_THROW(dut.SetState(kRuleId, kStateA, {kStateA}, {-kDurationUntil}), maliput::common::assertion_error);
  // Tries to set a nullopt next state with duration.
  EXPECT_THROW(dut.SetState(kRuleId, kStateA, {}, {kDurationUntil}), maliput::common::assertion_error);

  // Sets a valid state, next state and duration until.
  EXPECT_NO_THROW(dut.SetState(kRuleId, kStateA, {kStateB}, {kDurationUntil}));
  const std::optional<DiscreteValueRuleStateProvider::StateResult> result = dut.GetState(kRuleId);
  EXPECT_TRUE(result.has_value());
  EXPECT_TRUE(MALIPUT_IS_EQUAL(result->state, kStateA));
  EXPECT_TRUE(result->next.has_value());
  EXPECT_TRUE(MALIPUT_IS_EQUAL(result->next->state, kStateB));
  EXPECT_TRUE(result->next->duration_until.has_value());
  EXPECT_EQ(result->next->duration_until.value(), kDurationUntil);
}

TEST_F(ManualDiscreteRuleStateProviderTest, GetStateByRoadPositionAndRuleType) {
  ManualDiscreteValueRuleStateProvider dut(road_rulebook_.get());
  const api::test::MockLane lane{kLaneId};
  const RoadPosition road_position{
      &lane, LanePosition{(kLaneSRange.s_range().s0() + kLaneSRange.s_range().s1()) / 2., 0., 0.}};
  const double tolerance{1e-3};
  // Sets a valid state, next state and duration until.
  EXPECT_NO_THROW(dut.SetState(kRuleId, kStateA, {kStateB}, {kDurationUntil}));
  EXPECT_FALSE(dut.GetState(road_position, Rule::TypeId{"UnkownRuleType"}, tolerance).has_value());
  EXPECT_FALSE(dut.GetState({&lane, LanePosition{1000., 0., 0.}} /* Off zone */, kRuleType, tolerance).has_value());
  EXPECT_THROW(dut.GetState(road_position, kRuleType, -1. /* Negative tolerance */).has_value(),
               maliput::common::assertion_error);
  const std::optional<DiscreteValueRuleStateProvider::StateResult> result =
      dut.GetState(road_position, kRuleType, tolerance);
  EXPECT_TRUE(result.has_value());
  EXPECT_TRUE(MALIPUT_IS_EQUAL(result->state, kStateA));
  EXPECT_TRUE(result->next.has_value());
  EXPECT_TRUE(MALIPUT_IS_EQUAL(result->next->state, kStateB));
  EXPECT_TRUE(result->next->duration_until.has_value());
  EXPECT_EQ(result->next->duration_until.value(), kDurationUntil);
}

api::rules::Rule::RelatedRules CreateRightOfWayRelatedRules() {
  return std::map<std::string, std::vector<api::rules::Rule::Id>>{
      {VehicleStopInZoneBehaviorRuleTypeId().string(),
       {api::rules::Rule::Id{VehicleStopInZoneBehaviorRuleTypeId().string() + "/" + "test_id_a"}}},
      {RelatedRulesKeys::kYieldGroup,
       {api::rules::Rule::Id{RightOfWayRuleTypeId().string() + "/" + "test_id_b"},
        api::rules::Rule::Id{RightOfWayRuleTypeId().string() + "/" + "test_id_c"}}},
  };
}

class GetCurrentYieldGroupTest : public ::testing::Test {
 protected:
  const Rule::TypeId kTypeId{RightOfWayRuleTypeId()};
  const Rule::Id kRuleId{kTypeId.string() + "/right_of_way_rule_id"};
  const LaneSRoute kLaneSRoute{LaneSRoute{{LaneSRange{LaneId{"lane_id"}, {0., 10.}}}}};
  const DiscreteValueRule::DiscreteValue kStateDiscreteValue{DiscreteValueRule::DiscreteValue{
      Rule::State::kStrict, CreateRightOfWayRelatedRules(), api::test::CreateEmptyRelatedUniqueIds(), "StopAndGo"}};
  const std::vector<Rule::Id> expected_yield_group{CreateRightOfWayRelatedRules().at(RelatedRulesKeys::kYieldGroup)};

  void SetUp() override {
    road_rulebook_ = std::make_unique<ManualRulebook>();
    road_rulebook_->AddRule(DiscreteValueRule{kRuleId, kTypeId, kLaneSRoute, {kStateDiscreteValue}});
    discrete_value_rule_state_provider_ = std::make_unique<ManualDiscreteValueRuleStateProvider>(road_rulebook_.get());
    discrete_value_rule_state_provider_->SetState(kRuleId, kStateDiscreteValue, std::nullopt, std::nullopt);
  }

  std::unique_ptr<ManualRulebook> road_rulebook_;
  std::unique_ptr<maliput::ManualDiscreteValueRuleStateProvider> discrete_value_rule_state_provider_;
};

// Tests GetCurrentYieldGroup function.
TEST_F(GetCurrentYieldGroupTest, GetCurrentYieldGroup) {
  const std::vector<Rule::Id> dut{
      GetCurrentYieldGroup(road_rulebook_->GetDiscreteValueRule(kRuleId), discrete_value_rule_state_provider_.get())};

  EXPECT_EQ(dut.size(), expected_yield_group.size());
  for (const auto& expected_yield_id : expected_yield_group) {
    EXPECT_NE(std::find(dut.begin(), dut.end(), expected_yield_id), dut.end());
  }
}

api::rules::Rule::RelatedUniqueIds CreateRelatedUniqueIds() {
  return {
      {RelatedUniqueIdsKeys::kBulbGroup,
       {api::rules::UniqueBulbGroupId{maliput::api::rules::TrafficLight::Id{"traffic_light_a"},
                                      maliput::api::rules::BulbGroup::Id{"bulb_group_a"}},
        api::rules::UniqueBulbGroupId{maliput::api::rules::TrafficLight::Id{"traffic_light_b"},
                                      maliput::api::rules::BulbGroup::Id{"bulb_group_b"}}}},
  };
}

class GetCurrentBulbGroupTest : public ::testing::Test {
 protected:
  const Rule::TypeId kTypeId{RightOfWayRuleTypeId()};
  const Rule::Id kRuleId{kTypeId.string() + "/right_of_way_rule_id"};
  const DiscreteValueRule::DiscreteValue kStateDiscreteValue{DiscreteValueRule::DiscreteValue{
      Rule::State::kStrict, api::test::CreateEmptyRelatedRules(), CreateRelatedUniqueIds(), "StopAndGo"}};
  const std::vector<api::UniqueId> expected_bulb_group{CreateRelatedUniqueIds().at(RelatedUniqueIdsKeys::kBulbGroup)};

  void SetUp() override {
    road_rulebook_ = std::make_unique<ManualRulebook>();
    road_rulebook_->AddRule(DiscreteValueRule{kRuleId, kTypeId, api::test::CreateLaneSRoute(), {kStateDiscreteValue}});
    discrete_value_rule_state_provider_ = std::make_unique<ManualDiscreteValueRuleStateProvider>(road_rulebook_.get());
    discrete_value_rule_state_provider_->SetState(kRuleId, kStateDiscreteValue, std::nullopt, std::nullopt);
  }

  std::unique_ptr<ManualRulebook> road_rulebook_;
  std::unique_ptr<maliput::ManualDiscreteValueRuleStateProvider> discrete_value_rule_state_provider_;
};

// Tests GetCurrentBulbGroup function.
TEST_F(GetCurrentBulbGroupTest, GetCurrentBulbGroup) {
  const std::vector<api::UniqueId> dut{
      GetCurrentBulbGroup(road_rulebook_->GetDiscreteValueRule(kRuleId), discrete_value_rule_state_provider_.get())};

  EXPECT_EQ(dut.size(), expected_bulb_group.size());
  for (const auto& expected_bulb_id : expected_bulb_group) {
    EXPECT_NE(std::find(dut.begin(), dut.end(), expected_bulb_id), dut.end());
  }
}

}  // namespace
}  // namespace test
}  // namespace maliput
