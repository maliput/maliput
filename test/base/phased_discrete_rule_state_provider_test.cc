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
#include "maliput/base/phased_discrete_rule_state_provider.h"

#include <memory>

#include <gtest/gtest.h>

#include "maliput/api/lane.h"
#include "maliput/api/regions.h"
#include "maliput/api/rules/right_of_way_rule.h"
#include "maliput/api/rules/road_rulebook.h"
#include "maliput/base/manual_phase_provider.h"
#include "maliput/base/manual_phase_ring_book.h"
#include "maliput/base/manual_rulebook.h"
#include "maliput/base/rule_registry.h"
#include "maliput/common/assertion_error.h"
#include "maliput/test_utilities/mock.h"
#include "maliput/test_utilities/rules_compare.h"
#include "maliput/test_utilities/rules_test_utilities.h"

namespace maliput {
namespace {

using maliput::api::Lane;
using maliput::api::LaneId;
using maliput::api::LanePosition;
using maliput::api::LaneSRange;
using maliput::api::LaneSRoute;
using maliput::api::RoadPosition;
using maliput::api::rules::DiscreteValueRule;
using maliput::api::rules::DiscreteValueRuleStateProvider;
using maliput::api::rules::Phase;
using maliput::api::rules::PhaseRing;
using maliput::api::rules::RightOfWayRule;
using maliput::api::rules::RoadRulebook;
using maliput::api::rules::Rule;

// Evaluates constructor constraints.
GTEST_TEST(PhasedDiscreteRuleStateProviderTest, ConstructorConstraints) {
  const ManualRulebook rulebook;
  const ManualPhaseRingBook phase_ring_book;
  const ManualPhaseProvider phase_provider;

  EXPECT_THROW(PhasedDiscreteRuleStateProvider(nullptr, &phase_ring_book, &phase_provider),
               maliput::common::assertion_error);
  EXPECT_THROW(PhasedDiscreteRuleStateProvider(&rulebook, nullptr, &phase_provider), maliput::common::assertion_error);
  EXPECT_THROW(PhasedDiscreteRuleStateProvider(&rulebook, &phase_ring_book, nullptr), maliput::common::assertion_error);
  EXPECT_NO_THROW(PhasedDiscreteRuleStateProvider(&rulebook, &phase_ring_book, &phase_provider));
}

// Evaluates the phase based behavior of the state provider.
class PhaseBasedBehaviorTest : public ::testing::Test {
 protected:
  struct ExpectedStateByRule {
    const Rule::Id rule;
    const DiscreteValueRuleStateProvider::StateResult result;
  };

  struct ExpectedStateByRoadPosition {
    const maliput::api::RoadPosition road_position;
    const Rule::TypeId rule_type;
    const DiscreteValueRuleStateProvider::StateResult result;
  };

  const LaneId kLaneIdA{"a"};
  const LaneId kLaneIdB{"b"};
  const LaneSRange kZoneA{kLaneIdA, {0., 9.}};
  const LaneSRange kZoneB{kLaneIdB, {0., 9.}};
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  const RightOfWayRule::Id row_rule_id_a{"rule a"};
  const RightOfWayRule::Id row_rule_id_b{"rule b"};
  const RightOfWayRule::State::Id row_state_id_go{"GO"};
  const RightOfWayRule::State::Id row_state_id_stop{"STOP"};
  const RightOfWayRule right_of_way_rule_a{
      row_rule_id_a,
      LaneSRoute({kZoneA}),
      RightOfWayRule::ZoneType::kStopExcluded,
      {RightOfWayRule::State{row_state_id_go, RightOfWayRule::State::Type::kGo, {}},
       RightOfWayRule::State{row_state_id_stop, RightOfWayRule::State::Type::kStop, {}}},
      {} /* related_bulb_groups */
  };
  const RightOfWayRule right_of_way_rule_b{
      row_rule_id_b,
      LaneSRoute({kZoneB}),
      RightOfWayRule::ZoneType::kStopExcluded,
      {RightOfWayRule::State{row_state_id_go, RightOfWayRule::State::Type::kGo, {}},
       RightOfWayRule::State{row_state_id_stop, RightOfWayRule::State::Type::kStop, {}}},
      {} /* related_bulb_groups */
  };
#pragma GCC diagnostic pop
  const Rule::TypeId kRuleType{maliput::RightOfWayRuleTypeId()};
  const Rule::Id rule_id_a{kRuleType.string() + "/rule a"};
  const Rule::Id rule_id_b{kRuleType.string() + "/rule b"};
  const DiscreteValueRule::DiscreteValue state_go{Rule::State::kStrict, Rule::RelatedRules{}, Rule::RelatedUniqueIds{},
                                                  "Go"};
  const DiscreteValueRule::DiscreteValue state_stop{Rule::State::kStrict, Rule::RelatedRules{},
                                                    Rule::RelatedUniqueIds{}, "Stop"};
  const DiscreteValueRule right_of_way_discrete_value_rule_a{
      rule_id_a, kRuleType, LaneSRoute({kZoneA}), {state_go, state_stop}};
  const DiscreteValueRule right_of_way_discrete_value_rule_b{
      rule_id_b, kRuleType, LaneSRoute({kZoneB}), {state_go, state_stop}};

  const Phase::Id phase_id_1{"phase1"};
  const Phase phase1{phase_id_1,
                     {{row_rule_id_a, row_state_id_go}, {row_rule_id_b, row_state_id_stop}},
                     {{rule_id_a, state_go}, {rule_id_b, state_stop}}};
  const Phase::Id phase_id_2{"phase2"};
  const Phase phase2{phase_id_2,
                     {{row_rule_id_a, row_state_id_stop}, {row_rule_id_b, row_state_id_go}},
                     {{rule_id_a, state_stop}, {rule_id_b, state_go}}};

  const PhaseRing::Id ring_id{"ring"};
  const PhaseRing ring{ring_id, {phase1, phase2}};
  const double kTolerance{1e-3};

  void SetUp() override {
    rulebook_.AddRule(right_of_way_rule_a);
    rulebook_.AddRule(right_of_way_rule_b);
    rulebook_.AddRule(right_of_way_discrete_value_rule_a);
    rulebook_.AddRule(right_of_way_discrete_value_rule_b);

    phase_ring_book_.AddPhaseRing(ring);

    phase_provider_.AddPhaseRing(ring_id, phase_id_1);
  }

  void CompareExpectedTestCasesByRule(const DiscreteValueRuleStateProvider& dut,
                                      const std::vector<ExpectedStateByRule> test_cases) const {
    for (const auto& test : test_cases) {
      const std::optional<DiscreteValueRuleStateProvider::StateResult> result = dut.GetState(test.rule);
      EXPECT_TRUE(result.has_value());
      CompareDiscreteValueRuleStateProviderResult(test.result, *result);
    }
  }

  void CompareExpectedTestCasesByRoadPosition(const DiscreteValueRuleStateProvider& dut,
                                              const std::vector<ExpectedStateByRoadPosition> test_cases) const {
    for (const auto& test : test_cases) {
      const std::optional<DiscreteValueRuleStateProvider::StateResult> result =
          dut.GetState(test.road_position, test.rule_type, kTolerance);
      EXPECT_TRUE(result.has_value());
      CompareDiscreteValueRuleStateProviderResult(test.result, *result);
    }
  }

  void CompareDiscreteValueRuleStateProviderResult(
      const DiscreteValueRuleStateProvider::StateResult& expected_result,
      const DiscreteValueRuleStateProvider::StateResult& actual_result) const {
    EXPECT_EQ(expected_result.state, actual_result.state);
    if (expected_result.next) {
      EXPECT_EQ(expected_result.next->state, actual_result.next->state);
      if (expected_result.next->duration_until) {
        EXPECT_EQ(*expected_result.next->duration_until, *actual_result.next->duration_until);
      } else {
        EXPECT_EQ(expected_result.next->duration_until, std::nullopt);
      }
    } else {
      EXPECT_EQ(expected_result.next, std::nullopt);
    }
  }

  ManualRulebook rulebook_;
  ManualPhaseRingBook phase_ring_book_;
  ManualPhaseProvider phase_provider_;
};

TEST_F(PhaseBasedBehaviorTest, PhaseBasedBehavior) {
  PhasedDiscreteRuleStateProvider dut(&rulebook_, &phase_ring_book_, &phase_provider_);

  EXPECT_EQ(&dut.phase_ring_book(), &phase_ring_book_);
  EXPECT_EQ(&dut.phase_provider(), &phase_provider_);

  // TODO(liang.fok) Add tests for "next state" in returned results once #9993
  // is resolved.
  const std::vector<ExpectedStateByRule> phase_1_test_cases{{rule_id_a, {state_go, std::nullopt}},
                                                            {rule_id_b, {state_stop, std::nullopt}}};
  CompareExpectedTestCasesByRule(dut, phase_1_test_cases);
  phase_provider_.SetPhase(ring_id, phase_id_2);
  const std::vector<ExpectedStateByRule> phase_2_test_cases{{rule_id_a, {state_stop}}, {rule_id_b, {state_go}}};
  CompareExpectedTestCasesByRule(dut, phase_2_test_cases);
  EXPECT_FALSE(dut.GetState(Rule::Id("unknown rule")).has_value());
}

TEST_F(PhaseBasedBehaviorTest, GetStateByRoadPositionBehavior) {
  PhasedDiscreteRuleStateProvider dut(&rulebook_, &phase_ring_book_, &phase_provider_);
  EXPECT_EQ(&dut.phase_ring_book(), &phase_ring_book_);
  EXPECT_EQ(&dut.phase_provider(), &phase_provider_);

  const api::test::MockLane lane_a{kLaneIdA};
  const api::test::MockLane lane_b{kLaneIdB};
  const RoadPosition road_position_a{&lane_a,
                                     LanePosition{(kZoneA.s_range().s0() + kZoneA.s_range().s1()) / 2., 0., 0.}};
  const RoadPosition road_position_b{&lane_b,
                                     LanePosition{(kZoneB.s_range().s0() + kZoneB.s_range().s1()) / 2., 0., 0.}};
  const std::vector<ExpectedStateByRoadPosition> phase_1_test_cases{
      {road_position_a, kRuleType, {state_go, std::nullopt}},
      {road_position_b, kRuleType, {state_stop, std::nullopt}},
  };
  CompareExpectedTestCasesByRoadPosition(dut, phase_1_test_cases);
  phase_provider_.SetPhase(ring_id, phase_id_2);
  const std::vector<ExpectedStateByRoadPosition> phase_2_test_cases{
      {road_position_a, kRuleType, {state_stop, std::nullopt}},
      {road_position_b, kRuleType, {state_go, std::nullopt}},
  };
  CompareExpectedTestCasesByRoadPosition(dut, phase_2_test_cases);
  EXPECT_FALSE(dut.GetState({&lane_a, LanePosition{1000., 0., 0.}} /* Off zone */, kRuleType, kTolerance).has_value());
  EXPECT_THROW(dut.GetState(road_position_a, kRuleType, -1. /* Negative tolerance */).has_value(),
               maliput::common::assertion_error);
}

// Evaluates the manual behavior of the state provider.
class ManualBasedBehaviorTest : public ::testing::Test {
 protected:
  const Rule::Id kRuleId{"dvrt/dvr_id"};
  const Rule::Id kUnknownRuleId{"dvrt/unknown_id"};
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
  ManualPhaseRingBook phase_ring_book_;
  ManualPhaseProvider phase_provider_;
};

TEST_F(ManualBasedBehaviorTest, SetStateTest) {
  PhasedDiscreteRuleStateProvider dut(road_rulebook_.get(), &phase_ring_book_, &phase_provider_);

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

// Tests the states when using the PhasedDiscreteRuleStateProvider::GetDefaultPhasedDiscreteRuleStateProvider method.
TEST_F(ManualBasedBehaviorTest, StaticMethodTest) {
  // The discrete value rule that is added in the RoadRulebook is the one generated by
  // maliput::api::test::CreateDiscreteValueRule() helper method.
  const Rule::Id kRuleId{"dvrt/dvr_id"};
  const std::string kValue{"value1"};
  auto dut = PhasedDiscreteRuleStateProvider::GetDefaultPhasedDiscreteRuleStateProvider(
      road_rulebook_.get(), &phase_ring_book_, &phase_provider_);
  const auto state = dut->GetState(kRuleId);
  ASSERT_TRUE(state.has_value());
  EXPECT_EQ(kValue, state->state.value);
}

}  // namespace
}  // namespace maliput
