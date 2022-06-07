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
#include "maliput/base/manual_phase_ring_book.h"

#include <optional>
#include <stdexcept>

#include <gtest/gtest.h>

#include "maliput/api/rules/discrete_value_rule.h"
#include "maliput/api/rules/phase.h"
#include "maliput/api/rules/phase_ring.h"
#include "maliput/api/rules/right_of_way_rule.h"
#include "maliput/api/rules/rule.h"

namespace maliput {
namespace {

using api::rules::DiscreteValueRule;
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
        phase(Phase::Id("phase"),
              {{row_rule_id_a, RightOfWayRule::State::Id("a")}, {row_rule_id_b, RightOfWayRule::State::Id("b")}},
              {{rule_id_a, DiscreteValueRule::DiscreteValue{Rule::State::kStrict, Rule::RelatedRules{},
                                                            Rule::RelatedUniqueIds{}, "a"}},
               {rule_id_b, DiscreteValueRule::DiscreteValue{Rule::State::kStrict, Rule::RelatedRules{},
                                                            Rule::RelatedUniqueIds{}, "b"}}}),
        ring_id("ring"),
        ring(ring_id, {phase}) {}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  const RightOfWayRule::Id row_rule_id_a;
  const RightOfWayRule::Id row_rule_id_b;
#pragma GCC diagnostic pop
  const Rule::Id rule_id_a;
  const Rule::Id rule_id_b;
  const Phase phase;
  const PhaseRing::Id ring_id;
  const PhaseRing ring;
};

TEST_F(ManualPhaseRingBookTest, BasicTest) {
  ManualPhaseRingBook dut;
  EXPECT_NO_THROW(dut.AddPhaseRing(ring));
  EXPECT_EQ(static_cast<int>(dut.GetPhaseRings().size()), 1);
  EXPECT_EQ(dut.GetPhaseRings()[0], ring.id());
  std::optional<PhaseRing> result = dut.GetPhaseRing(ring_id);
  EXPECT_TRUE(result.has_value());
  EXPECT_EQ(result->id(), ring_id);
  const PhaseRing::Id unknown_ring_id("unknown ring");
  EXPECT_EQ(dut.GetPhaseRing(unknown_ring_id), std::nullopt);
  for (const auto rule_id : {rule_id_a, rule_id_b}) {
    result = dut.FindPhaseRing(rule_id);
    EXPECT_TRUE(result.has_value());
    EXPECT_EQ(result->id(), ring_id);
  }
  const Rule::Id unknown_rule_id("unknown rule");
  EXPECT_EQ(dut.FindPhaseRing(unknown_rule_id), std::nullopt);
  EXPECT_THROW(dut.RemovePhaseRing(unknown_ring_id), std::logic_error);
  EXPECT_NO_THROW(dut.RemovePhaseRing(ring_id));
  EXPECT_EQ(dut.GetPhaseRing(ring_id), std::nullopt);
  for (const auto rule_id : {rule_id_a, rule_id_b}) {
    EXPECT_EQ(dut.FindPhaseRing(rule_id), std::nullopt);
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
      {{rule_id_c,
        DiscreteValueRule::DiscreteValue{Rule::State::kStrict, Rule::RelatedRules{}, Rule::RelatedUniqueIds{}, "c"}}});
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
      {{rule_id_a,
        DiscreteValueRule::DiscreteValue{Rule::State::kStrict, Rule::RelatedRules{}, Rule::RelatedUniqueIds{}, "a"}}});
  const PhaseRing ring_with_overlapping_rule(PhaseRing::Id("unique phase ID"), {phase_with_overlapping_rule});
  EXPECT_THROW(dut.AddPhaseRing(ring_with_overlapping_rule), std::logic_error);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
// TODO(agalbachicar)  This test should be removed once RightOfWayRules are.
TEST_F(ManualPhaseRingBookTest, RightOfWayRuleApiTest) {
  ManualPhaseRingBook dut;

  EXPECT_NO_THROW(dut.AddPhaseRing(ring));

  const PhaseRing::Id unknown_ring_id("unknown ring");
  EXPECT_EQ(dut.GetPhaseRing(unknown_ring_id), std::nullopt);

  for (const auto rule_id : {row_rule_id_a, row_rule_id_b}) {
    const std::optional<PhaseRing> result = dut.FindPhaseRing(rule_id);
    EXPECT_TRUE(result.has_value());
    EXPECT_EQ(result->id(), ring_id);
  }
  const RightOfWayRule::Id unknown_row_rule_id("unknown rule");
  EXPECT_EQ(dut.FindPhaseRing(unknown_row_rule_id), std::nullopt);
  EXPECT_THROW(dut.RemovePhaseRing(unknown_ring_id), std::logic_error);
  EXPECT_NO_THROW(dut.RemovePhaseRing(ring_id));
  EXPECT_EQ(dut.GetPhaseRing(ring_id), std::nullopt);
  for (const auto rule_id : {row_rule_id_a, row_rule_id_b}) {
    EXPECT_EQ(dut.FindPhaseRing(rule_id), std::nullopt);
  }
}
#pragma GCC diagnostic pop

// @ }

}  // namespace
}  // namespace maliput
