#include "maliput/base/manual_phase_ring_book.h"

#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/common/drake_optional.h"

#include "maliput/api/rules/discrete_value_rule.h"
#include "maliput/api/rules/phase.h"
#include "maliput/api/rules/phase_ring.h"
#include "maliput/api/rules/right_of_way_rule.h"
#include "maliput/api/rules/rule.h"

namespace maliput {
namespace {

using api::rules::MakeDiscreteValue;
using api::rules::Phase;
using api::rules::PhaseRing;
using api::rules::RightOfWayRule;
using api::rules::Rule;

struct ManualPhaseRingBookTest : public ::testing::Test {
  ManualPhaseRingBookTest()
      : row_rule_id_a("rule a"),
        row_rule_id_b("rule b"),
        rule_id_a("rule a"),
        rule_id_b("rule b"),
        phase(
            Phase::Id("phase"),
            {{row_rule_id_a, RightOfWayRule::State::Id("a")}, {row_rule_id_b, RightOfWayRule::State::Id("b")}},
            {{rule_id_a, MakeDiscreteValue(Rule::State::kStrict, Rule::RelatedRules{}, Rule::RelatedUniqueIds{}, "a")},
             {rule_id_b,
              MakeDiscreteValue(Rule::State::kStrict, Rule::RelatedRules{}, Rule::RelatedUniqueIds{}, "b")}}),
        ring_id("ring"),
        ring(ring_id, {phase}) {}

  const RightOfWayRule::Id row_rule_id_a;
  const RightOfWayRule::Id row_rule_id_b;
  const Rule::Id rule_id_a;
  const Rule::Id rule_id_b;
  const Phase phase;
  const PhaseRing::Id ring_id;
  const PhaseRing ring;
};

TEST_F(ManualPhaseRingBookTest, BasicTest) {
  ManualPhaseRingBook dut;
  EXPECT_NO_THROW(dut.AddPhaseRing(ring));
  EXPECT_EQ(dut.GetPhaseRings().size(), 1);
  EXPECT_EQ(dut.GetPhaseRings()[0], ring.id());
  drake::optional<PhaseRing> result = dut.GetPhaseRing(ring_id);
  EXPECT_TRUE(result.has_value());
  EXPECT_EQ(result->id(), ring_id);
  const PhaseRing::Id unknown_ring_id("unknown ring");
  EXPECT_EQ(dut.GetPhaseRing(unknown_ring_id), drake::nullopt);
  for (const auto rule_id : {rule_id_a, rule_id_b}) {
    result = dut.FindPhaseRing(rule_id);
    EXPECT_TRUE(result.has_value());
    EXPECT_EQ(result->id(), ring_id);
  }
  const Rule::Id unknown_rule_id("unknown rule");
  EXPECT_EQ(dut.FindPhaseRing(unknown_rule_id), drake::nullopt);
  EXPECT_THROW(dut.RemovePhaseRing(unknown_ring_id), std::logic_error);
  EXPECT_NO_THROW(dut.RemovePhaseRing(ring_id));
  EXPECT_EQ(dut.GetPhaseRing(ring_id), drake::nullopt);
  for (const auto rule_id : {rule_id_a, rule_id_b}) {
    EXPECT_EQ(dut.FindPhaseRing(rule_id), drake::nullopt);
  }
}

// Verifies that an exception is thrown when the user attempts to add a
// different PhaseRing that has the same ID as a previously added PhaseRing.
TEST_F(ManualPhaseRingBookTest, RingWithSameId) {
  ManualPhaseRingBook dut;
  dut.AddPhaseRing(ring);
  const RightOfWayRule::Id row_rule_id_c("rule c");
  const Rule::Id rule_id_c("rule c");
  const Phase different_phase(
      Phase::Id("different phase with different rules"), {{row_rule_id_c, RightOfWayRule::State::Id("c")}},
      {{rule_id_c, MakeDiscreteValue(Rule::State::kStrict, Rule::RelatedRules{}, Rule::RelatedUniqueIds{}, "c")}});
  const PhaseRing ring_with_same_id(ring_id, {different_phase});
  EXPECT_THROW(dut.AddPhaseRing(ring_with_same_id), std::logic_error);
}

// Verifies that an exception is thrown when the user attempts to add a
// PhaseRing with a unique ID but contains a phase with a RightOfWayRule::Id /
// Rule::Id that overlaps the rules covered by a previously added PhaseRing.
TEST_F(ManualPhaseRingBookTest, RingWithOverlappingRule) {
  ManualPhaseRingBook dut;
  dut.AddPhaseRing(ring);
  const Phase phase_with_overlapping_rule(
      Phase::Id("different phase with overlapping rules"), {{row_rule_id_a, RightOfWayRule::State::Id("a")}},
      {{rule_id_a, MakeDiscreteValue(Rule::State::kStrict, Rule::RelatedRules{}, Rule::RelatedUniqueIds{}, "a")}});
  const PhaseRing ring_with_overlapping_rule(PhaseRing::Id("unique phase ID"), {phase_with_overlapping_rule});
  EXPECT_THROW(dut.AddPhaseRing(ring_with_overlapping_rule), std::logic_error);
}

// TODO(agalbachicar)  This test should be removed once RightOfWayRules are.
TEST_F(ManualPhaseRingBookTest, RightOfWayRuleApiTest) {
  ManualPhaseRingBook dut;

  EXPECT_NO_THROW(dut.AddPhaseRing(ring));

  const PhaseRing::Id unknown_ring_id("unknown ring");
  EXPECT_EQ(dut.GetPhaseRing(unknown_ring_id), drake::nullopt);

  for (const auto rule_id : {row_rule_id_a, row_rule_id_b}) {
    const drake::optional<PhaseRing> result = dut.FindPhaseRing(rule_id);
    EXPECT_TRUE(result.has_value());
    EXPECT_EQ(result->id(), ring_id);
  }
  const RightOfWayRule::Id unknown_row_rule_id("unknown rule");
  EXPECT_EQ(dut.FindPhaseRing(unknown_row_rule_id), drake::nullopt);
  EXPECT_THROW(dut.RemovePhaseRing(unknown_ring_id), std::logic_error);
  EXPECT_NO_THROW(dut.RemovePhaseRing(ring_id));
  EXPECT_EQ(dut.GetPhaseRing(ring_id), drake::nullopt);
  for (const auto rule_id : {row_rule_id_a, row_rule_id_b}) {
    EXPECT_EQ(dut.FindPhaseRing(rule_id), drake::nullopt);
  }
}

// @ }

}  // namespace
}  // namespace maliput
