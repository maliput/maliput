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
#include "maliput/base/phase_based_right_of_way_rule_state_provider.h"

#include <gtest/gtest.h>

#include "maliput/base/manual_phase_provider.h"
#include "maliput/base/manual_phase_ring_book.h"

namespace maliput {
namespace {

using maliput::api::rules::DiscreteValueRule;
using maliput::api::rules::Phase;
using maliput::api::rules::PhaseRing;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
using maliput::api::rules::RightOfWayRule;
using maliput::api::rules::RightOfWayRuleStateProvider;
#pragma GCC diagnostic pop
using maliput::api::rules::Rule;

GTEST_TEST(PhaseBasedRightOfWayRuleStateProviderTest, BasicTest) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  const RightOfWayRule::Id row_rule_id_a("rule a");
  const RightOfWayRule::Id row_rule_id_b("rule b");
  const RightOfWayRule::State::Id row_state_id_go("GO");
  const RightOfWayRule::State::Id row_state_id_stop("STOP");
#pragma GCC diagnostic pop

  const Rule::Id rule_id_a("Right-Of-Way/rule a");
  const Rule::Id rule_id_b("Right-Of-Way/rule b");
  const DiscreteValueRule::DiscreteValue state_go =
      DiscreteValueRule::DiscreteValue{Rule::State::kStrict, Rule::RelatedRules{}, Rule::RelatedUniqueIds{}, "Go"};
  const DiscreteValueRule::DiscreteValue state_stop =
      DiscreteValueRule::DiscreteValue{Rule::State::kStrict, Rule::RelatedRules{}, Rule::RelatedUniqueIds{}, "Stop"};

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

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  PhaseBasedRightOfWayRuleStateProvider dut(&phase_ring_book, &phase_provider);
#pragma GCC diagnostic pop

  EXPECT_EQ(&dut.phase_ring_book(), &phase_ring_book);
  EXPECT_EQ(&dut.phase_provider(), &phase_provider);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  struct ExpectedState {
    const RightOfWayRule::Id rule;
    const RightOfWayRuleStateProvider::RightOfWayResult result;
  };

  auto compare_expected = [&](const std::vector<ExpectedState>& test_cases) {
    for (const auto& test : test_cases) {
      const std::optional<RightOfWayRuleStateProvider::RightOfWayResult> result = dut.GetState(test.rule);
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
  };

  // TODO(liang.fok) Add tests for "next state" in returned results once #9993
  // is resolved.
  const std::vector<ExpectedState> phase_1_test_cases{{row_rule_id_a, {row_state_id_go, std::nullopt}},
                                                      {row_rule_id_b, {row_state_id_stop, std::nullopt}}};
  compare_expected(phase_1_test_cases);
  phase_provider.SetPhase(ring_id, phase_id_2);
  const std::vector<ExpectedState> phase_2_test_cases{{row_rule_id_a, {row_state_id_stop}},
                                                      {row_rule_id_b, {row_state_id_go}}};
  compare_expected(phase_2_test_cases);
  EXPECT_FALSE(dut.GetState(RightOfWayRule::Id("unknown rule")).has_value());
#pragma GCC diagnostic pop
}

}  // namespace
}  // namespace maliput
