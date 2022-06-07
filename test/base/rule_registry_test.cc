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
#include "maliput/base/rule_registry.h"

#include <algorithm>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "maliput/api/rules/discrete_value_rule.h"
#include "maliput/api/rules/rule.h"

namespace maliput {
namespace test {
namespace {

GTEST_TEST(DirectionUsageRuleTypeIdTest, Initialization) {
  EXPECT_EQ(DirectionUsageRuleTypeId().string(), "Direction-Usage Rule Type");
}

GTEST_TEST(RightOfWayRuleTypeIdTest, Initialization) {
  EXPECT_EQ(RightOfWayRuleTypeId().string(), "Right-Of-Way Rule Type");
}

GTEST_TEST(VehicleStopInZoneBehaviorRuleTypeIdTest, Initialization) {
  EXPECT_EQ(VehicleStopInZoneBehaviorRuleTypeId().string(), "Vehicle-Stop-In-Zone-Behavior Rule Type");
}

GTEST_TEST(SpeedLimitRuleTypeIdTest, Initialization) {
  EXPECT_EQ(SpeedLimitRuleTypeId().string(), "Speed-Limit Rule Type");
}

// Holds the information to evaluate the rule type built by `builder` function.
struct BuildDiscreteValueRuleTypeExpectedValues {
  std::string type_id;
  std::vector<int> severities;
  std::vector<std::string> values;
  std::function<api::rules::DiscreteValueRuleTypeAndValues()> builder;
};

// Tests build rule type functions.
class BuildDiscreteValueRuleTypeTest : public ::testing::TestWithParam<BuildDiscreteValueRuleTypeExpectedValues> {
 protected:
  void SetUp() override { expectation_ = GetParam(); }

  bool HaveDiscreteValueWith(const std::vector<api::rules::DiscreteValueRule::DiscreteValue>& values, int severity,
                             const std::string& value) {
    const auto discrete_value = api::rules::DiscreteValueRule::DiscreteValue{
        severity, {} /* related_rules */, {} /* related_unique_ids */, value};
    return std::find(values.begin(), values.end(), discrete_value) != values.end();
  }

  BuildDiscreteValueRuleTypeExpectedValues expectation_;
};

std::vector<BuildDiscreteValueRuleTypeExpectedValues> BuildDiscreteValueRuleTypeTestParameters() {
  return {
      {DirectionUsageRuleTypeId().string(),
       {api::rules::Rule::State::kStrict},
       {"WithS", "AgainstS", "Bidirectional", "BidirectionalTurnOnly", "NoUse", "Parking", "Undefined"},
       BuildDirectionUsageRuleType},
      {RightOfWayRuleTypeId().string(),
       {api::rules::Rule::State::kStrict, api::rules::Rule::State::kBestEffort},
       {"Go", "Stop", "StopAndGo"},
       BuildRightOfWayRuleType},
      {VehicleStopInZoneBehaviorRuleTypeId().string(),
       {api::rules::Rule::State::kStrict},
       {"DoNotStop", "5MinuteParking", "30MinuteParking", "45MinuteParking", "1HourParking", "2HourParking",
        "4HourParking", "UnconstrainedParking"},
       BuildVehicleStopInZoneBehaviorRuleType},
  };
}

TEST_P(BuildDiscreteValueRuleTypeTest, EvaluateRuleTypes) {
  const api::rules::DiscreteValueRuleTypeAndValues dut = expectation_.builder();

  EXPECT_EQ(dut.first, api::rules::Rule::TypeId(expectation_.type_id));
  for (const int severity : expectation_.severities) {
    for (const std::string& discrete_value : expectation_.values) {
      EXPECT_TRUE(HaveDiscreteValueWith(dut.second, severity, discrete_value));
    }
  }
}

INSTANTIATE_TEST_CASE_P(BuildDiscreteValueRuleTypeTestGroup, BuildDiscreteValueRuleTypeTest,
                        ::testing::ValuesIn(BuildDiscreteValueRuleTypeTestParameters()));

}  // namespace
}  // namespace test
}  // namespace maliput
