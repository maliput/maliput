#include "maliput/base/phase_based_right_of_way_discrete_value_rule_state_provider.h"

#include <gtest/gtest.h>

#include "maliput/base/manual_phase_provider.h"
#include "maliput/base/manual_phase_ring_book.h"

namespace maliput {
namespace {

using maliput::api::rules::DiscreteValueRule;
using maliput::api::rules::DiscreteValueRuleStateProvider;
using maliput::api::rules::MakeDiscreteValue;
using maliput::api::rules::Phase;
using maliput::api::rules::PhaseRing;
using maliput::api::rules::RightOfWayRule;
using maliput::api::rules::Rule;

GTEST_TEST(PhaseBasedRightOfWayDiscreteValueRuleStateProviderTest, BasicTest) {
  const RightOfWayRule::Id row_rule_id_a("rule a");
  const RightOfWayRule::Id row_rule_id_b("rule b");
  const RightOfWayRule::State::Id row_state_id_go("GO");
  const RightOfWayRule::State::Id row_state_id_stop("STOP");

  const Rule::Id rule_id_a("Right-Of-Way/rule a");
  const Rule::Id rule_id_b("Right-Of-Way/rule b");
  const DiscreteValueRule::DiscreteValue state_go =
      MakeDiscreteValue(Rule::State::kStrict, Rule::RelatedRules{}, Rule::RelatedUniqueIds{}, "Go");
  const DiscreteValueRule::DiscreteValue state_stop =
      MakeDiscreteValue(Rule::State::kStrict, Rule::RelatedRules{}, Rule::RelatedUniqueIds{}, "Stop");

  const Phase::Id phase_id_1("phase1");
  const Phase phase1(phase_id_1, {{row_rule_id_a, row_state_id_go}, {row_rule_id_b, row_state_id_stop}},
                     {{rule_id_a, state_go}, {rule_id_b, state_stop}});
  const Phase::Id phase_id_2("phase2");
  const Phase phase2(phase_id_2, {{row_rule_id_a, row_state_id_stop}, {row_rule_id_b, row_state_id_go}},
                     {{rule_id_a, state_stop}, {rule_id_b, state_go}});

  const PhaseRing::Id ring_id("ring");
  const PhaseRing ring(ring_id, {phase1, phase2});

  ManualPhaseRingBook phase_ring_book;
  phase_ring_book.AddPhaseRing(ring);

  ManualPhaseProvider phase_provider;
  phase_provider.AddPhaseRing(ring_id, phase_id_1);

  PhaseBasedRightOfWayDiscreteValueRuleStateProvider dut(&phase_ring_book, &phase_provider);

  EXPECT_EQ(&dut.phase_ring_book(), &phase_ring_book);
  EXPECT_EQ(&dut.phase_provider(), &phase_provider);

  struct ExpectedState {
    const Rule::Id rule;
    const DiscreteValueRuleStateProvider::StateResult result;
  };

  auto compare_expected = [&](const std::vector<ExpectedState>& test_cases) {
    for (const auto& test : test_cases) {
      drake::optional<DiscreteValueRuleStateProvider::StateResult> result = dut.GetState(test.rule);
      EXPECT_TRUE(result.has_value());
      EXPECT_EQ(result->state, test.result.state);
      if (test.result.next) {
        EXPECT_EQ(result->next->state, test.result.next->state);
        if (test.result.next->duration_until) {
          EXPECT_EQ(*result->next->duration_until, *test.result.next->duration_until);
        } else {
          EXPECT_EQ(result->next->duration_until, drake::nullopt);
        }
      } else {
        EXPECT_EQ(result->next, drake::nullopt);
      }
    }
  };

  // TODO(liang.fok) Add tests for "next state" in returned results once #9993
  // is resolved.
  const std::vector<ExpectedState> phase_1_test_cases{{rule_id_a, {state_go, drake::nullopt}},
                                                      {rule_id_b, {state_stop, drake::nullopt}}};
  compare_expected(phase_1_test_cases);
  phase_provider.SetPhase(ring_id, phase_id_2);
  const std::vector<ExpectedState> phase_2_test_cases{{rule_id_a, {state_stop}}, {rule_id_b, {state_go}}};
  compare_expected(phase_2_test_cases);
  EXPECT_FALSE(dut.GetState(Rule::Id("unknown rule")).has_value());
}

}  // namespace
}  // namespace maliput
