#include "maliput/base/phase_based_right_of_way_discrete_value_rule_state_provider.h"

#include <memory>

#include <gtest/gtest.h>

#include "maliput/api/lane.h"
#include "maliput/api/regions.h"
#include "maliput/api/rules/right_of_way_rule.h"
#include "maliput/api/rules/road_rulebook.h"
#include "maliput/base/manual_phase_provider.h"
#include "maliput/base/manual_phase_ring_book.h"
#include "maliput/base/manual_rulebook.h"
#include "maliput/common/assertion_error.h"
#include "maliput/test_utilities/mock.h"
#include "maliput/test_utilities/rules_compare.h"
#include "maliput/test_utilities/rules_test_utilities.h"

namespace maliput {
namespace {

using maliput::api::LaneId;
using maliput::api::LaneSRange;
using maliput::api::LaneSRoute;
using maliput::api::rules::DiscreteValueRule;
using maliput::api::rules::DiscreteValueRuleStateProvider;
using maliput::api::rules::Phase;
using maliput::api::rules::PhaseRing;
using maliput::api::rules::RightOfWayRule;
using maliput::api::rules::RoadRulebook;
using maliput::api::rules::Rule;

// Evaluates constructor constraints.
GTEST_TEST(PhaseBasedRightOfWayDiscreteValueRuleStateProviderTest, ConstructorConstraints) {
  const ManualRulebook rulebook;
  const ManualPhaseRingBook phase_ring_book;
  const ManualPhaseProvider phase_provider;

  EXPECT_THROW(PhaseBasedRightOfWayDiscreteValueRuleStateProvider(nullptr, &phase_ring_book, &phase_provider),
               maliput::common::assertion_error);
  EXPECT_THROW(PhaseBasedRightOfWayDiscreteValueRuleStateProvider(&rulebook, nullptr, &phase_provider),
               maliput::common::assertion_error);
  EXPECT_THROW(PhaseBasedRightOfWayDiscreteValueRuleStateProvider(&rulebook, &phase_ring_book, nullptr),
               maliput::common::assertion_error);
  EXPECT_NO_THROW(PhaseBasedRightOfWayDiscreteValueRuleStateProvider(&rulebook, &phase_ring_book, &phase_provider));
}

// Evaluates the phase based behavior of the state provider.
class PhaseBasedBehaviorTest : public ::testing::Test {
 protected:
  struct ExpectedState {
    const Rule::Id rule;
    const DiscreteValueRuleStateProvider::StateResult result;
  };

  const LaneSRange kZone{LaneId{"a"}, {10., 20.}};
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  const RightOfWayRule::Id row_rule_id_a{"rule a"};
  const RightOfWayRule::Id row_rule_id_b{"rule b"};
  const RightOfWayRule::State::Id row_state_id_go{"GO"};
  const RightOfWayRule::State::Id row_state_id_stop{"STOP"};
  const RightOfWayRule right_of_way_rule_a{
      row_rule_id_a,
      LaneSRoute({kZone}),
      RightOfWayRule::ZoneType::kStopExcluded,
      {RightOfWayRule::State{row_state_id_go, RightOfWayRule::State::Type::kGo, {}},
       RightOfWayRule::State{row_state_id_stop, RightOfWayRule::State::Type::kStop, {}}},
      {} /* related_bulb_groups */
  };
  const RightOfWayRule right_of_way_rule_b{
      row_rule_id_b,
      LaneSRoute({kZone}),
      RightOfWayRule::ZoneType::kStopExcluded,
      {RightOfWayRule::State{row_state_id_go, RightOfWayRule::State::Type::kGo, {}},
       RightOfWayRule::State{row_state_id_stop, RightOfWayRule::State::Type::kStop, {}}},
      {} /* related_bulb_groups */
  };
#pragma GCC diagnostic pop

  const Rule::Id rule_id_a{"Right-Of-Way/rule a"};
  const Rule::Id rule_id_b{"Right-Of-Way/rule b"};
  const DiscreteValueRule::DiscreteValue state_go{Rule::State::kStrict, Rule::RelatedRules{}, Rule::RelatedUniqueIds{},
                                                  "Go"};
  const DiscreteValueRule::DiscreteValue state_stop{Rule::State::kStrict, Rule::RelatedRules{},
                                                    Rule::RelatedUniqueIds{}, "Stop"};
  const DiscreteValueRule right_of_way_discrete_value_rule_a{
      rule_id_a, Rule::TypeId("Right-Of-Way"), LaneSRoute({kZone}), {state_go, state_stop}};
  const DiscreteValueRule right_of_way_discrete_value_rule_b{
      rule_id_b, Rule::TypeId("Right-Of-Way"), LaneSRoute({kZone}), {state_go, state_stop}};

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

  void SetUp() override {
    rulebook_.AddRule(right_of_way_rule_a);
    rulebook_.AddRule(right_of_way_rule_b);
    rulebook_.AddRule(right_of_way_discrete_value_rule_a);
    rulebook_.AddRule(right_of_way_discrete_value_rule_b);

    phase_ring_book_.AddPhaseRing(ring);

    phase_provider_.AddPhaseRing(ring_id, phase_id_1);
  }

  void CompareExpectedTestCases(const DiscreteValueRuleStateProvider& dut,
                                const std::vector<ExpectedState>& test_cases) const {
    for (const auto& test : test_cases) {
      const std::optional<DiscreteValueRuleStateProvider::StateResult> result = dut.GetState(test.rule);
      EXPECT_TRUE(result.has_value());
      EXPECT_EQ(result->state, test.result.state);
      if (test.result.next) {
        EXPECT_EQ(result->next->state, test.result.next->state);
        if (test.result.next->duration_until) {
          EXPECT_EQ(*result->next->duration_until, *test.result.next->duration_until);
        } else {
          EXPECT_EQ(result->next->duration_until, std::nullopt);
        }
      } else {
        EXPECT_EQ(result->next, std::nullopt);
      }
    }
  }

  ManualRulebook rulebook_;
  ManualPhaseRingBook phase_ring_book_;
  ManualPhaseProvider phase_provider_;
};

TEST_F(PhaseBasedBehaviorTest, PhaseBasedBehavior) {
  PhaseBasedRightOfWayDiscreteValueRuleStateProvider dut(&rulebook_, &phase_ring_book_, &phase_provider_);

  EXPECT_EQ(&dut.phase_ring_book(), &phase_ring_book_);
  EXPECT_EQ(&dut.phase_provider(), &phase_provider_);

  // TODO(liang.fok) Add tests for "next state" in returned results once #9993
  // is resolved.
  const std::vector<ExpectedState> phase_1_test_cases{{rule_id_a, {state_go, std::nullopt}},
                                                      {rule_id_b, {state_stop, std::nullopt}}};
  CompareExpectedTestCases(dut, phase_1_test_cases);
  phase_provider_.SetPhase(ring_id, phase_id_2);
  const std::vector<ExpectedState> phase_2_test_cases{{rule_id_a, {state_stop}}, {rule_id_b, {state_go}}};
  CompareExpectedTestCases(dut, phase_2_test_cases);
  EXPECT_FALSE(dut.GetState(Rule::Id("unknown rule")).has_value());
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
  PhaseBasedRightOfWayDiscreteValueRuleStateProvider dut(road_rulebook_.get(), &phase_ring_book_, &phase_provider_);

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

}  // namespace
}  // namespace maliput
