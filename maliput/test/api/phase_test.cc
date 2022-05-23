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
#include "maliput/api/rules/phase.h"
/* clang-format on */
// TODO(liang.fok) Satisfy clang-format via rules tests directory reorg.

#include <vector>

#include <gtest/gtest.h>

#include "maliput/common/assertion_error.h"
#include "maliput/test_utilities/phases_compare.h"
#include "maliput/test_utilities/rules_test_utilities.h"

namespace maliput {
namespace api {
namespace rules {
namespace {

class PhaseTest : public ::testing::Test {
 protected:
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  PhaseTest()
      : id_("test_id"),
        rule_states_{{RightOfWayRule::Id("northbound-forward"), RightOfWayRule::State::Id("GO")},
                     {RightOfWayRule::Id("southbound-left-turn"), RightOfWayRule::State::Id("STOP")}},
        discrete_value_rule_states_{{Rule::Id("RightOfWayRuleType/northbound-forward"),
                                     DiscreteValueRule::DiscreteValue{Rule::State::kStrict, Rule::RelatedRules{},
                                                                      Rule::RelatedUniqueIds{}, "Go"}},
                                    {Rule::Id("RightOfWayRuleType/southbound-left-turn"),
                                     DiscreteValueRule::DiscreteValue{Rule::State::kStrict, Rule::RelatedRules{},
                                                                      Rule::RelatedUniqueIds{}, "Stop"}}},
        bulb_states_{
            {{{TrafficLight::Id("major-intersection"), BulbGroup::Id("northbound"), Bulb::Id("forward-green")},
              BulbState::kOn},
             {{TrafficLight::Id("major-intersection"), BulbGroup::Id("northbound"), Bulb::Id("forward-red")},
              BulbState::kOff},
             {{TrafficLight::Id("major-intersection"), BulbGroup::Id("southbound"), Bulb::Id("left-turn-green")},
              BulbState::kOff},
             {{TrafficLight::Id("major-intersection"), BulbGroup::Id("southbound"), Bulb::Id("left-turn-red")},
              BulbState::kOn}}},
        phase_{id_, rule_states_, discrete_value_rule_states_, bulb_states_} {}
#pragma GCC diagnostic pop

  const Phase::Id id_;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  const RuleStates rule_states_;
#pragma GCC diagnostic pop
  const DiscreteValueRuleStates discrete_value_rule_states_;
  const std::optional<BulbStates> bulb_states_;
  const Phase phase_;
};

TEST_F(PhaseTest, Accessors) {
  for (const std::optional<BulbStates>& bulb_states :
       std::vector<std::optional<BulbStates>>{std::nullopt, bulb_states_}) {
    Phase dut(id_, rule_states_, discrete_value_rule_states_, bulb_states);
    EXPECT_EQ(dut.id(), id_);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    EXPECT_EQ(dut.rule_states(), rule_states_);
#pragma GCC diagnostic pop
    EXPECT_EQ(dut.discrete_value_rule_states(), discrete_value_rule_states_);
    EXPECT_EQ(dut.bulb_states(), bulb_states);
  }
}

TEST_F(PhaseTest, Copying) {
  const Phase dut(phase_);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, phase_));
}

TEST_F(PhaseTest, Assignment) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  Phase dut(Phase::Id("other_dut_id"), RuleStates(), discrete_value_rule_states_);
#pragma GCC diagnostic pop
  dut = phase_;
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, phase_));
}

}  // namespace
}  // namespace rules
}  // namespace api
}  // namespace maliput
