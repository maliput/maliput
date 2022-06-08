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
#include "maliput/api/rules/discrete_value_rule_state_provider.h"

#include <gtest/gtest.h>

#include "maliput/api/rules/discrete_value_rule.h"
#include "maliput/api/rules/rule.h"
#include "maliput/test_utilities/mock.h"
#include "maliput/test_utilities/rules_compare.h"
#include "maliput/test_utilities/rules_test_utilities.h"

namespace maliput {
namespace api {
namespace rules {
namespace {

// Mock class to evaluate DiscreteValueRuleStateProvider interface.
class MockDiscreteValueRuleStateProvider : public DiscreteValueRuleStateProvider {
 public:
  static const double kTolerance;
  static const Rule::Id kRuleId;
  static const Rule::TypeId kRuleType;
  static const RoadPosition kRoadPosition;
  static DiscreteValueRule::DiscreteValue MakeCurrentDiscreteValue();
  static DiscreteValueRule::DiscreteValue MakeNextDiscreteValue();

 private:
  std::optional<StateResult> DoGetState(const Rule::Id& id) const override {
    if (id == kRuleId) {
      return StateResult{MakeCurrentDiscreteValue(),
                         StateResult::Next{MakeNextDiscreteValue(), 123.456 /* duration */}};
    }
    return {};
  }
  std::optional<StateResult> DoGetState(const RoadPosition& road_position, const Rule::TypeId& rule_type,
                                        double tolerance) const override {
    if (road_position.lane == kRoadPosition.lane && road_position.pos.srh() == kRoadPosition.pos.srh() &&
        rule_type == kRuleType && tolerance == kTolerance) {
      return StateResult{MakeCurrentDiscreteValue(),
                         StateResult::Next{MakeNextDiscreteValue(), 123.456 /* duration */}};
    }
    return {};
  }
};

const double MockDiscreteValueRuleStateProvider::kTolerance{1e-3};
const Rule::Id MockDiscreteValueRuleStateProvider::kRuleId{"RuleId"};
// Using nullptr lane for the sake of the test.
const RoadPosition MockDiscreteValueRuleStateProvider::kRoadPosition{nullptr, LanePosition{0., 0., 0.}};
const Rule::TypeId MockDiscreteValueRuleStateProvider::kRuleType{"My-Rule-Type"};

DiscreteValueRule::DiscreteValue MockDiscreteValueRuleStateProvider::MakeCurrentDiscreteValue() {
  return DiscreteValueRule::DiscreteValue{Rule::State::kStrict, maliput::api::test::CreateEmptyRelatedRules(),
                                          maliput::api::test::CreateEmptyRelatedUniqueIds(), "current_state"};
}

DiscreteValueRule::DiscreteValue MockDiscreteValueRuleStateProvider::MakeNextDiscreteValue() {
  return DiscreteValueRule::DiscreteValue{Rule::State::kStrict, maliput::api::test::CreateEmptyRelatedRules(),
                                          maliput::api::test::CreateEmptyRelatedUniqueIds(), "next_state"};
}

GTEST_TEST(DiscreteValueRuleStateProviderTest, GetStateById) {
  const MockDiscreteValueRuleStateProvider dut;
  const std::optional<DiscreteValueRuleStateProvider::StateResult> result =
      dut.GetState(MockDiscreteValueRuleStateProvider::kRuleId);

  EXPECT_TRUE(result.has_value());
  EXPECT_TRUE(MALIPUT_IS_EQUAL(result->state, MockDiscreteValueRuleStateProvider::MakeCurrentDiscreteValue()));
  EXPECT_TRUE(result->next.has_value());
  EXPECT_TRUE(MALIPUT_IS_EQUAL(result->next->state, MockDiscreteValueRuleStateProvider::MakeNextDiscreteValue()));
  EXPECT_TRUE(result->next->duration_until.has_value());
  EXPECT_EQ(result->next->duration_until.value(), 123.456);

  EXPECT_FALSE(dut.GetState(Rule::Id("UnregisteredRule")).has_value());
}

GTEST_TEST(DiscreteValueRuleStateProviderTest, GetStateByRoadPositionAndType) {
  const MockDiscreteValueRuleStateProvider dut;
  const std::optional<DiscreteValueRuleStateProvider::StateResult> result =
      dut.GetState(MockDiscreteValueRuleStateProvider::kRoadPosition, MockDiscreteValueRuleStateProvider::kRuleType,
                   MockDiscreteValueRuleStateProvider::kTolerance);

  EXPECT_TRUE(result.has_value());
  EXPECT_TRUE(MALIPUT_IS_EQUAL(result->state, MockDiscreteValueRuleStateProvider::MakeCurrentDiscreteValue()));
  EXPECT_TRUE(result->next.has_value());
  EXPECT_TRUE(MALIPUT_IS_EQUAL(result->next->state, MockDiscreteValueRuleStateProvider::MakeNextDiscreteValue()));
  EXPECT_TRUE(result->next->duration_until.has_value());
  EXPECT_EQ(result->next->duration_until.value(), 123.456);

  EXPECT_FALSE(dut.GetState(RoadPosition{nullptr, LanePosition{100., 0., 0.}},
                            MockDiscreteValueRuleStateProvider::kRuleType,
                            MockDiscreteValueRuleStateProvider::kTolerance)
                   .has_value());
  EXPECT_FALSE(dut.GetState(MockDiscreteValueRuleStateProvider::kRoadPosition, Rule::TypeId("Unkown Rule Type"),
                            MockDiscreteValueRuleStateProvider::kTolerance)
                   .has_value());
  EXPECT_FALSE(dut.GetState(MockDiscreteValueRuleStateProvider::kRoadPosition,
                            MockDiscreteValueRuleStateProvider::kRuleType, 800)
                   .has_value());
}

}  // namespace
}  // namespace rules
}  // namespace api
}  // namespace maliput
