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
#include "maliput/api/rules/direction_usage_rule.h"
/* clang-format on */
// TODO(andrew.best@tri.global) Satisfy clang-format via rules tests
//                              directory reorg.

#include <utility>

#include <gtest/gtest.h>

#include "assert_compare.h"
#include "maliput/api/compare.h"
#include "maliput/api/regions.h"
#include "maliput/api/rules/compare.h"

namespace maliput {
namespace api {
namespace rules {
namespace {

using maliput::test::AssertCompare;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
using Severity = DirectionUsageRule::State::Severity;
using Type = DirectionUsageRule::State::Type;

class DirectionUsageTest : public ::testing::Test {
 protected:
  LaneSRange kZone{LaneId("the_lane"), SRange(13., 15.)};

  const DirectionUsageRule MakeDefaultRule() const {
    return DirectionUsageRule(DirectionUsageRule::Id("default_rule"), kZone,
                              std::vector<DirectionUsageRule::State>{DirectionUsageRule::State(
                                  DirectionUsageRule::State::Id("some_state"), Type::kWithS, Severity::kStrict)});
  }

  const DirectionUsageRule MakeFromSeverityAndType(Severity severity, Type type) const {
    const DirectionUsageRule::State the_state(DirectionUsageRule::State::Id("some_state"), type, severity);
    const std::vector<DirectionUsageRule::State> states{the_state};
    return DirectionUsageRule(DirectionUsageRule::Id("severity_type_rule"), kZone, states);
  }

  // Provides an example of each Type and alternates Severity for iteration.
  // Does not imply coverage on the entire set of combinations, but provides
  // at least one example of each Type and Severity.
  const std::vector<std::pair<Severity, Type>> test_rules_{
      {Severity::kStrict, Type::kWithS},         {Severity::kPreferred, Type::kAgainstS},
      {Severity::kStrict, Type::kBidirectional}, {Severity::kPreferred, Type::kNoUse},
      {Severity::kStrict, Type::kParking},       {Severity::kPreferred, Type::kBidirectionalTurnOnly},
  };
};

TEST_F(DirectionUsageTest, Construction) {
  for (const auto rule_config : test_rules_) {
    const DirectionUsageRule::State the_state(DirectionUsageRule::State::Id("some_state"), rule_config.second,
                                              rule_config.first);
    const std::vector<DirectionUsageRule::State> states{the_state};

    EXPECT_NO_THROW(DirectionUsageRule(DirectionUsageRule::Id("some_id"), kZone, states));
  }
}

TEST_F(DirectionUsageTest, AccessCopyAssign) {
  for (const auto rule_config : test_rules_) {
    const DirectionUsageRule source = MakeFromSeverityAndType(rule_config.first, rule_config.second);
    const DirectionUsageRule dut1(source);
    DirectionUsageRule dut2 = MakeDefaultRule();
    // Verify copy and assign.
    EXPECT_EQ(source.id(), dut1.id());
    EXPECT_NE(source.id(), dut2.id());
    dut2 = source;
    EXPECT_TRUE(AssertCompare(IsEqual(source, dut2)));
    EXPECT_TRUE(AssertCompare(IsEqual(source, dut1)));

    EXPECT_TRUE(AssertCompare(IsEqual(source.zone(), dut1.zone())));
    EXPECT_TRUE(AssertCompare(IsEqual(source.is_static(), dut2.is_static())));
    EXPECT_TRUE(AssertCompare(IsEqual(source.static_state(), dut2.static_state())));
    EXPECT_TRUE(AssertCompare(IsEqual(source.static_state().type(), dut1.static_state().type())));
    EXPECT_TRUE(AssertCompare(IsEqual(source.static_state().severity(), dut2.static_state().severity())));
  }
}

TEST_F(DirectionUsageTest, StateTypeMapperTest) {
  const auto dut = DirectionUsageRule::StateTypeMapper();
  const std::vector<DirectionUsageRule::State::Type> expected_types{
      DirectionUsageRule::State::Type::kWithS,         DirectionUsageRule::State::Type::kAgainstS,
      DirectionUsageRule::State::Type::kBidirectional, DirectionUsageRule::State::Type::kBidirectionalTurnOnly,
      DirectionUsageRule::State::Type::kNoUse,         DirectionUsageRule::State::Type::kParking};
  EXPECT_EQ(dut.size(), expected_types.size());
  for (DirectionUsageRule::State::Type type : expected_types) {
    EXPECT_EQ(static_cast<int>(dut.count(type)), 1);
  }
}
#pragma GCC diagnostic pop

}  // namespace
}  // namespace rules
}  // namespace api
}  // namespace maliput
