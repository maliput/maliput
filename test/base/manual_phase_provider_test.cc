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
#include "maliput/base/manual_phase_ring_book.h"
#include "maliput/test_utilities/mock.h"

namespace maliput {
namespace {

using api::rules::BulbStates;
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

// Tests the ManualPhaseProvider::GetDefaultPopulatedManualPhaseProvider method.
// For this, two phase_rings are added to the phase ring book:
// 1 - Containing 3 phases with next phases information.
// 2 - Containing 3 phases without next phases information.
class DefaultPopulatedManualPhaseProviderTest : public testing::Test {
 public:
  static DiscreteValueRule::DiscreteValue CreateDiscreteValue(const std::string& value) {
    return {Rule::State::kStrict, api::test::CreateEmptyRelatedRules(), api::test::CreateEmptyRelatedUniqueIds(),
            value};
  }
  static constexpr double kDurationUntil = 10.0;

 protected:
  void SetUp() override {
    phase_ring_book_.AddPhaseRing(phase_ring_with_next_);
    phase_ring_book_.AddPhaseRing(phase_ring_without_next_);
  }

  const Phase::Id kPhaseId1{"phase_1"};
  const Phase::Id kPhaseId2{"phase_2"};
  const Phase::Id kPhaseId3{"phase_3"};
  const Phase::Id kPhaseId4{"phase_4"};
  const Phase::Id kPhaseId5{"phase_5"};
  const Phase::Id kPhaseId6{"phase_6"};
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  const RuleStates kEmptyRuleStates{};
#pragma GCC diagnostic pop
  const BulbStates kEmptyBulbStates{};
  const Phase phase_1_{kPhaseId1,
                       kEmptyRuleStates,
                       {
                           {Rule::Id{"rule_a"}, CreateDiscreteValue("rule_a_value_1")},
                           {Rule::Id{"rule_b"}, CreateDiscreteValue("rule_b_value_1")},
                       } /* discrete_value_rule_states */,
                       kEmptyBulbStates};

  const Phase phase_2_{kPhaseId2,
                       kEmptyRuleStates,
                       {
                           {Rule::Id{"rule_a"}, CreateDiscreteValue("rule_a_value_2")},
                           {Rule::Id{"rule_b"}, CreateDiscreteValue("rule_b_value_2")},
                       } /* discrete_value_rule_states */,
                       kEmptyBulbStates};
  const Phase phase_3_{kPhaseId3,
                       kEmptyRuleStates,
                       {
                           {Rule::Id{"rule_a"}, CreateDiscreteValue("rule_a_value_3")},
                           {Rule::Id{"rule_b"}, CreateDiscreteValue("rule_b_value_3")},
                       } /* discrete_value_rule_states */,
                       kEmptyBulbStates};
  const Phase phase_4_{kPhaseId4,
                       kEmptyRuleStates,
                       {
                           {Rule::Id{"rule_c"}, CreateDiscreteValue("rule_c_value_1")},
                           {Rule::Id{"rule_d"}, CreateDiscreteValue("rule_d_value_1")},
                       } /* discrete_value_rule_states */,
                       kEmptyBulbStates};

  const Phase phase_5_{kPhaseId5,
                       kEmptyRuleStates,
                       {
                           {Rule::Id{"rule_c"}, CreateDiscreteValue("rule_c_value_2")},
                           {Rule::Id{"rule_d"}, CreateDiscreteValue("rule_d_value_2")},
                       } /* discrete_value_rule_states */,
                       kEmptyBulbStates};
  const Phase phase_6_{kPhaseId6,
                       kEmptyRuleStates,
                       {
                           {Rule::Id{"rule_c"}, CreateDiscreteValue("rule_c_value_3")},
                           {Rule::Id{"rule_d"}, CreateDiscreteValue("rule_d_value_3")},
                       } /* discrete_value_rule_states */,
                       kEmptyBulbStates};
  const PhaseRing::Id phase_ring_id_1_{"phase_ring_1"};
  const PhaseRing::Id phase_ring_id_2_{"phase_ring_2"};
  const PhaseRing phase_ring_with_next_{phase_ring_id_1_,
                                        {phase_1_, phase_2_, phase_3_},
                                        {{
                                            {phase_1_.id(), {{phase_2_.id(), {kDurationUntil}}}},
                                            {phase_2_.id(), {{phase_3_.id(), {kDurationUntil}}}},
                                            {phase_3_.id(), {{phase_1_.id(), {kDurationUntil}}}},
                                        }}};
  const PhaseRing phase_ring_without_next_{phase_ring_id_2_, {phase_4_, phase_5_, phase_6_}};

  ManualPhaseRingBook phase_ring_book_{};
};

TEST_F(DefaultPopulatedManualPhaseProviderTest, Test) {
  const std::unique_ptr<ManualPhaseProvider> dut{
      ManualPhaseProvider::GetDefaultPopulatedManualPhaseProvider(&phase_ring_book_)};

  // Phase ring book 1.
  const std::optional<PhaseProvider::Result> phase_from_ring_1 = dut->GetPhase(phase_ring_id_1_);
  ASSERT_TRUE(phase_from_ring_1.has_value());
  // As the phases aren't added in order, we need to check if the gotten phase is actually part of the ring.
  ASSERT_TRUE(phase_ring_with_next_.GetPhase(phase_from_ring_1->state).has_value());
  ASSERT_TRUE(phase_from_ring_1->next.has_value());
  // Once we get the phase we could check the next phase.
  if (phase_from_ring_1->state == kPhaseId1) {
    EXPECT_EQ(kPhaseId3, phase_from_ring_1->next->state);
  } else if (phase_from_ring_1->state == kPhaseId2) {
    EXPECT_EQ(kPhaseId3, phase_from_ring_1->next->state);
  } else if (phase_from_ring_1->state == kPhaseId3) {
    EXPECT_EQ(kPhaseId1, phase_from_ring_1->next->state);
  } else {
    FAIL();
  }
  EXPECT_EQ(phase_from_ring_1->next->duration_until, kDurationUntil);

  // Phase ring book 2.
  const std::optional<PhaseProvider::Result> phase_from_ring_2 = dut->GetPhase(phase_ring_id_2_);
  ASSERT_TRUE(phase_from_ring_2.has_value());
  ASSERT_TRUE(phase_ring_without_next_.GetPhase(phase_from_ring_2->state).has_value());
  ASSERT_FALSE(phase_from_ring_2->next.has_value());
}

}  // namespace
}  // namespace maliput
