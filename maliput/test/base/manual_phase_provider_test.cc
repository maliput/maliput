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
#include "maliput/base/manual_phase_provider.h"

#include <optional>
#include <stdexcept>

#include <gtest/gtest.h>

#include "maliput/api/rules/discrete_value_rule.h"
#include "maliput/api/rules/phase.h"
#include "maliput/api/rules/phase_provider.h"
#include "maliput/api/rules/phase_ring.h"
#include "maliput/api/rules/right_of_way_rule.h"
#include "maliput/api/rules/rule.h"

namespace maliput {
namespace {

using api::rules::DiscreteValueRule;
using api::rules::DiscreteValueRuleStates;
using api::rules::Phase;
using api::rules::PhaseProvider;
using api::rules::PhaseRing;
using api::rules::RightOfWayRule;
using api::rules::Rule;
using api::rules::RuleStates;

constexpr double kDurationUntil{10.};  // Arbitrarily chosen.

// Fixture for testing the ManualPhaseProvider.
struct ManualPhaseProviderTest : public ::testing::Test {
  ManualPhaseProviderTest() : phase_id_1("foo"), phase_id_2("bar"), phase_ring_id("bar") {}

  const Phase::Id phase_id_1;
  const Phase::Id phase_id_2;
  const PhaseRing::Id phase_ring_id;
  ManualPhaseProvider dut;
};

TEST_F(ManualPhaseProviderTest, EmptyProvider) {
  EXPECT_EQ(dut.GetPhase(phase_ring_id), std::nullopt);
  EXPECT_THROW(dut.SetPhase(phase_ring_id, phase_id_1), std::exception);
}

// Tests that an exception is thrown if duration-until is specified but the next
// phase is not when adding a PhaseRing.
TEST_F(ManualPhaseProviderTest, AddPhaseRingDurationSpecifiedOnly) {
  EXPECT_THROW(dut.AddPhaseRing(phase_ring_id, phase_id_1, std::nullopt /* next phase */, kDurationUntil),
               std::logic_error);
}

// Tests that an exception is thrown if duration-until is specified but the next
// phase is not when setting the phase of a PhaseRing.
TEST_F(ManualPhaseProviderTest, SetPhaseRingDurationSpecifiedOnly) {
  EXPECT_NO_THROW(dut.AddPhaseRing(phase_ring_id, phase_id_1));
  EXPECT_THROW(dut.SetPhase(phase_ring_id, phase_id_1, std::nullopt /* next phase */, kDurationUntil),
               std::logic_error);
}

// Tests situation where only the current phase is specified.
TEST_F(ManualPhaseProviderTest, CurrentPhaseOnly) {
  EXPECT_NO_THROW(dut.AddPhaseRing(phase_ring_id, phase_id_1));
  EXPECT_THROW(dut.AddPhaseRing(phase_ring_id, phase_id_1), std::logic_error);
  std::optional<PhaseProvider::Result> returned_phase = dut.GetPhase(phase_ring_id);
  EXPECT_EQ(returned_phase->state, phase_id_1);
  EXPECT_EQ(returned_phase->next, std::nullopt);
  EXPECT_NO_THROW(dut.SetPhase(phase_ring_id, phase_id_2));
  returned_phase = dut.GetPhase(phase_ring_id);
  EXPECT_EQ(returned_phase->state, phase_id_2);
  EXPECT_EQ(returned_phase->next, std::nullopt);
}

// Tests situation where both the current and next phases are specified.
TEST_F(ManualPhaseProviderTest, CurrentAndNextPhases) {
  EXPECT_NO_THROW(dut.AddPhaseRing(phase_ring_id, phase_id_1, phase_id_2, kDurationUntil));
  std::optional<PhaseProvider::Result> returned_phase = dut.GetPhase(phase_ring_id);
  EXPECT_EQ(returned_phase->state, phase_id_1);
  EXPECT_NE(returned_phase->next, std::nullopt);
  EXPECT_EQ(returned_phase->next->state, phase_id_2);
  EXPECT_EQ(*returned_phase->next->duration_until, kDurationUntil);
  EXPECT_NO_THROW(dut.SetPhase(phase_ring_id, phase_id_2, phase_id_1, 2 * kDurationUntil));
  returned_phase = dut.GetPhase(phase_ring_id);
  EXPECT_EQ(returned_phase->state, phase_id_2);
  EXPECT_NE(returned_phase->next, std::nullopt);
  EXPECT_EQ(returned_phase->next->state, phase_id_1);
  EXPECT_EQ(*returned_phase->next->duration_until, 2 * kDurationUntil);
}

// Tests that an exception is thrown if the phases within a PhaseRing cover
// different sets of RightOfWayRules.
GTEST_TEST(PhaseRingTest, InvalidPhases) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

  const RuleStates rule_states_1{{RightOfWayRule::Id("a"), RightOfWayRule::State::Id("1")},
                                 {RightOfWayRule::Id("b"), RightOfWayRule::State::Id("2")}};
  const RuleStates rule_states_2{{RightOfWayRule::Id("a"), RightOfWayRule::State::Id("1")}};
#pragma GCC diagnostic pop

  const DiscreteValueRuleStates discrete_value_rule_states_1{
      {Rule::Id("a"),
       {DiscreteValueRule::DiscreteValue{Rule::State::kStrict, Rule::RelatedRules{}, Rule::RelatedUniqueIds{}, "1"}}},
      {Rule::Id("b"),
       {DiscreteValueRule::DiscreteValue{Rule::State::kStrict, Rule::RelatedRules{}, Rule::RelatedUniqueIds{}, "2"}}}};
  const DiscreteValueRuleStates discrete_value_rule_states_2{
      {Rule::Id("a"),
       {DiscreteValueRule::DiscreteValue{Rule::State::kStrict, Rule::RelatedRules{}, Rule::RelatedUniqueIds{}, "1"}}}};

  Phase phase_1(Phase::Id("bar"), rule_states_1, discrete_value_rule_states_1);
  Phase phase_2(Phase::Id("baz"), rule_states_2, discrete_value_rule_states_2);
  const std::vector<Phase> phases{phase_1, phase_2};
  EXPECT_THROW(PhaseRing(PhaseRing::Id("foo"), phases), std::exception);
}

}  // namespace
}  // namespace maliput
