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
/* clang-format off to disable clang-format-includes */
#include "maliput/api/rules/phase_ring.h"
/* clang-format on */
// TODO(liang.fok) Satisfy clang-format via rules tests directory reorg.

#include <exception>
#include <stdexcept>

#include <gtest/gtest.h>

#include "maliput/api/rules/discrete_value_rule.h"
#include "maliput/api/rules/phase.h"
#include "maliput/api/rules/right_of_way_rule.h"
#include "maliput/api/rules/rule.h"
#include "maliput/api/rules/traffic_lights.h"
#include "maliput/test_utilities/phases_compare.h"
#include "maliput/test_utilities/rules_test_utilities.h"

namespace maliput {
namespace api {
namespace rules {
namespace {

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

Phase CreateFullPhase(const Phase::Id& id) {
  return Phase{
      id,
      {{RightOfWayRule::Id("rule_a"), RightOfWayRule::State::Id("GO")},
       {RightOfWayRule::Id("rule_b"), RightOfWayRule::State::Id("STOP")}},
      {{Rule::Id("RightOfWayRuleType/rule_a"),
        DiscreteValueRule::DiscreteValue{Rule::State::kStrict, Rule::RelatedRules{}, Rule::RelatedUniqueIds{}, "Go"}},
       {Rule::Id("RightOfWayRuleType/rule_b"),
        DiscreteValueRule::DiscreteValue{Rule::State::kStrict, Rule::RelatedRules{}, Rule::RelatedUniqueIds{},
                                         "Stop"}}},
      {{{{TrafficLight::Id("my_intersection"), BulbGroup::Id("my_bulb_group"), Bulb::Id("rule_a_green")},
         BulbState::kOn},
        {{TrafficLight::Id("my_intersection"), BulbGroup::Id("my_bulb_group"), Bulb::Id("rule_a_red")},
         BulbState::kOff},
        {{TrafficLight::Id("my_intersection"), BulbGroup::Id("my_bulb_group"), Bulb::Id("rule_b_green")},
         BulbState::kOff},
        {{TrafficLight::Id("my_intersection"), BulbGroup::Id("my_bulb_group"), Bulb::Id("rule_b_red")},
         BulbState::kOn}}}};
}
#pragma GCC diagnostic pop

Phase CreatePhaseWithMissingRuleStates(const Phase::Id& id) {
  const Phase mock_phase = CreateFullPhase(id);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  RuleStates rule_states = mock_phase.rule_states();
#pragma GCC diagnostic pop
  rule_states.erase(rule_states.begin());
  DiscreteValueRuleStates discrete_value_rule_states = mock_phase.discrete_value_rule_states();
  discrete_value_rule_states.erase(discrete_value_rule_states.begin());
  return Phase(id, rule_states, discrete_value_rule_states, mock_phase.bulb_states());
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
Phase CreatePhaseWithMissingBulbStates(const Phase::Id& id) {
  const Phase mock_phase = CreateFullPhase(id);
  BulbStates bulb_states = *mock_phase.bulb_states();
  bulb_states.erase(bulb_states.begin());
  return Phase(id, mock_phase.rule_states(), mock_phase.discrete_value_rule_states(), bulb_states);
}
#pragma GCC diagnostic pop

class PhaseRingTest : public ::testing::Test {
 protected:
  PhaseRingTest() : id_("my_ring"), phase_id_1_("phase_1"), phase_id_2_("phase_2") {}

  const PhaseRing::Id id_;
  const Phase::Id phase_id_1_;
  const Phase::Id phase_id_2_;
};

TEST_F(PhaseRingTest, Constructor) {
  EXPECT_NO_THROW(PhaseRing(id_, {CreateFullPhase(phase_id_1_), CreateFullPhase(phase_id_2_)}));

  // No phases.
  EXPECT_THROW(PhaseRing(id_, {}), std::exception);

  // Duplicate phases.
  EXPECT_THROW(PhaseRing(id_, {CreateFullPhase(phase_id_1_), CreateFullPhase(phase_id_1_)}), std::exception);

  // Phases that differ in RightOfWayRule coverage.
  EXPECT_THROW(PhaseRing(id_, {CreateFullPhase(phase_id_1_), CreatePhaseWithMissingRuleStates(phase_id_2_)}),
               std::exception);

  // Phases that differ in bulb state coverage.
  EXPECT_THROW(PhaseRing(id_, {CreateFullPhase(phase_id_1_), CreatePhaseWithMissingBulbStates(phase_id_2_)}),
               std::exception);

  // Next phases does not fully cover phases.
  const std::unordered_map<Phase::Id, std::vector<PhaseRing::NextPhase>> partial_next_phases = {
      {phase_id_1_, std::vector<PhaseRing::NextPhase>()}};
  EXPECT_THROW(PhaseRing(id_, {CreateFullPhase(phase_id_1_), CreateFullPhase(phase_id_2_)}, partial_next_phases),
               std::exception);

  // Next phases defines an unknown phase.
  const std::unordered_map<Phase::Id, std::vector<PhaseRing::NextPhase>> unknown_next_phases = {
      {Phase::Id("unknown"), std::vector<PhaseRing::NextPhase>()}};
  EXPECT_THROW(PhaseRing(id_, {CreateFullPhase(phase_id_1_), CreateFullPhase(phase_id_2_)}, unknown_next_phases),
               std::exception);
}

TEST_F(PhaseRingTest, Accessors) {
  const PhaseRing dut(id_, {CreateFullPhase(phase_id_1_), CreateFullPhase(phase_id_2_)});
  EXPECT_EQ(dut.id(), id_);
  EXPECT_EQ(static_cast<int>(dut.phases().size()), 2);
  EXPECT_EQ(static_cast<int>(dut.next_phases().size()), 2);
  EXPECT_EQ(static_cast<int>(dut.next_phases().at(phase_id_1_).size()), 0);
  EXPECT_EQ(static_cast<int>(dut.next_phases().at(phase_id_2_).size()), 0);
}

TEST_F(PhaseRingTest, GetPhase) {
  const PhaseRing dut(id_, {CreateFullPhase(phase_id_1_), CreateFullPhase(phase_id_2_)});
  std::optional<Phase> phase_result = dut.GetPhase(phase_id_1_);
  EXPECT_TRUE(phase_result.has_value());
  EXPECT_EQ(phase_result->id(), phase_id_1_);

  phase_result = dut.GetPhase(Phase::Id("UnknownPhase"));
  EXPECT_FALSE(phase_result.has_value());
}

TEST_F(PhaseRingTest, NextPhases) {
  const double kDuration1{30};
  const double kDuration2{60};
  std::unordered_map<Phase::Id, std::vector<PhaseRing::NextPhase>> next_phases;
  next_phases.emplace(std::make_pair(phase_id_1_, std::vector<PhaseRing::NextPhase>{{phase_id_2_, kDuration1}}));
  next_phases.emplace(std::make_pair(phase_id_2_, std::vector<PhaseRing::NextPhase>{{phase_id_1_, kDuration2}}));
  const PhaseRing dut(id_, {CreateFullPhase(phase_id_1_), CreateFullPhase(phase_id_2_)}, next_phases);
  const std::vector<PhaseRing::NextPhase> next_1 = dut.next_phases().at(phase_id_1_);
  const std::vector<PhaseRing::NextPhase> next_2 = dut.next_phases().at(phase_id_2_);
  EXPECT_EQ(static_cast<int>(next_1.size()), 1);
  EXPECT_EQ(static_cast<int>(next_2.size()), 1);
  EXPECT_EQ(next_1.at(0).id, phase_id_2_);
  EXPECT_EQ(next_2.at(0).id, phase_id_1_);
  EXPECT_EQ(*next_1.at(0).duration_until, kDuration1);
  EXPECT_EQ(*next_2.at(0).duration_until, kDuration2);
  EXPECT_EQ(dut.GetNextPhases(phase_id_1_).at(0).id, phase_id_2_);
  EXPECT_EQ(dut.GetNextPhases(phase_id_2_).at(0).id, phase_id_1_);
  EXPECT_THROW(dut.GetNextPhases(Phase::Id("Unknown")), std::out_of_range);
}

}  // namespace
}  // namespace rules
}  // namespace api
}  // namespace maliput
