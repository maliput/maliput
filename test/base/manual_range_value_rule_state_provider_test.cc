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
#include "maliput/base/manual_range_value_rule_state_provider.h"

#include <stdexcept>

#include <gtest/gtest.h>

#include "maliput/api/rules/range_value_rule.h"
#include "maliput/api/rules/rule.h"
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
using api::rules::RangeValueRule;
using api::rules::RangeValueRuleStateProvider;
using api::rules::RoadRulebook;
using api::rules::Rule;

class ManualRangeValueRuleStateProviderTest : public ::testing::Test {
 protected:
  const Rule::Id kRuleId{"rvrt/rvr_id"};
  const Rule::TypeId kRuleType{"rvrt"};
  const Rule::Id kUnknownRuleId{"rvrt/unknown_id"};
  const LaneId kLaneId{"a"};
  const LaneSRange kLaneSRange{kLaneId, {0., 9.}};
  const RangeValueRule::Range kRangeA{api::test::CreateRange()};
  const RangeValueRule::Range kInvalidRange{
      RangeValueRule::Range{Rule::State::kStrict, api::test::CreateEmptyRelatedRules(),
                            api::test::CreateEmptyRelatedUniqueIds(), "invalid", 456. /* min */, 789. /* max */}};
  const double kDurationUntil{10.};

  void SetUp() override {
    const maliput::api::test::RoadRulebookBuildFlags kRulebookBuildFlags{
        false /* add_right_of_way */, {} /* right_of_way_build_flags */,   false /* add_direction_usage */,
        false /* add_speed_limit */,  false /* add_discrete_value_rule */, true /* add_range_value_rule */};
    road_rulebook_ = api::test::CreateRoadRulebook(kRulebookBuildFlags);
  }

  std::unique_ptr<RoadRulebook> road_rulebook_;
};

TEST_F(ManualRangeValueRuleStateProviderTest, ConstructorConstraints) {
  EXPECT_THROW(ManualRangeValueRuleStateProvider(nullptr), maliput::common::assertion_error);
  EXPECT_NO_THROW(ManualRangeValueRuleStateProvider(road_rulebook_.get()));
}

TEST_F(ManualRangeValueRuleStateProviderTest, SetStateTest) {
  ManualRangeValueRuleStateProvider dut(road_rulebook_.get());

  // Tries to set the state to an unknown Rule::Id in `rulebook_`.
  EXPECT_THROW(dut.SetState(kUnknownRuleId, kRangeA, {}, {}), std::out_of_range);
  // Tries to set an invalid state to the rule.
  EXPECT_THROW(dut.SetState(kRuleId, kInvalidRange, {}, {}), maliput::common::assertion_error);
  // Tries to set an invalid next state to the rule.
  EXPECT_THROW(dut.SetState(kRuleId, kRangeA, {kInvalidRange}, {}), maliput::common::assertion_error);
  // Tries to set a valid next state with a negative duration.
  EXPECT_THROW(dut.SetState(kRuleId, kRangeA, {kRangeA}, {-kDurationUntil}), maliput::common::assertion_error);
  // Tries to set a nullopt next state with duration.
  EXPECT_THROW(dut.SetState(kRuleId, kRangeA, {}, {kDurationUntil}), maliput::common::assertion_error);

  // Sets a valid state, next state and duration until.
  EXPECT_NO_THROW(dut.SetState(kRuleId, kRangeA, {kRangeA}, {kDurationUntil}));
  const std::optional<RangeValueRuleStateProvider::StateResult> result = dut.GetState(kRuleId);
  EXPECT_TRUE(result.has_value());
  EXPECT_TRUE(MALIPUT_IS_EQUAL(result->state, kRangeA));
  EXPECT_TRUE(result->next.has_value());
  EXPECT_TRUE(MALIPUT_IS_EQUAL(result->next->state, kRangeA));
  EXPECT_TRUE(result->next->duration_until.has_value());
  EXPECT_EQ(result->next->duration_until.value(), kDurationUntil);
}

TEST_F(ManualRangeValueRuleStateProviderTest, GetStateByRoadPositionAndRuleType) {
  ManualRangeValueRuleStateProvider dut(road_rulebook_.get());
  const api::test::MockLane lane{kLaneId};
  const RoadPosition road_position{
      &lane, LanePosition{(kLaneSRange.s_range().s0() + kLaneSRange.s_range().s1()) / 2., 0., 0.}};
  const double tolerance{1e-3};
  // Sets a valid state, next state and duration until.
  EXPECT_NO_THROW(dut.SetState(kRuleId, kRangeA, {kRangeA}, {kDurationUntil}));
  EXPECT_FALSE(dut.GetState(road_position, Rule::TypeId{"UnkownRuleType"}, tolerance).has_value());
  EXPECT_FALSE(dut.GetState({&lane, LanePosition{1000., 0., 0.}} /* Off zone */, kRuleType, tolerance).has_value());
  EXPECT_THROW(dut.GetState(road_position, kRuleType, -1. /* Negative tolerance */).has_value(),
               maliput::common::assertion_error);
  const std::optional<RangeValueRuleStateProvider::StateResult> result =
      dut.GetState(road_position, kRuleType, tolerance);
  EXPECT_TRUE(result.has_value());
  EXPECT_TRUE(MALIPUT_IS_EQUAL(result->state, kRangeA));
  EXPECT_TRUE(result->next.has_value());
  EXPECT_TRUE(MALIPUT_IS_EQUAL(result->next->state, kRangeA));
  EXPECT_TRUE(result->next->duration_until.has_value());
  EXPECT_EQ(result->next->duration_until.value(), kDurationUntil);
}

// Tests the states when using the ManualRangeValueRuleStateProvider::GetDefaultManualRangeValueRuleStateProvider
// method.
TEST_F(ManualRangeValueRuleStateProviderTest, StaticMethodTest) {
  // The discrete value rule that is added in the RoadRulebook is the one generated by
  // maliput::api::test::CreateRangeValueRule() helper method.
  const Rule::Id kRuleId{"rvrt/rvr_id"};
  const std::string kValue{"value1"};
  auto dut = ManualRangeValueRuleStateProvider::GetDefaultManualRangeValueRuleStateProvider(road_rulebook_.get());
  const auto state = dut->GetState(kRuleId);
  ASSERT_TRUE(state.has_value());
  EXPECT_EQ(123., state->state.min);
  EXPECT_EQ(456., state->state.max);
}

}  // namespace
}  // namespace test
}  // namespace maliput
