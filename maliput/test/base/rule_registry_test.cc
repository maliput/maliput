#include "maliput/base/rule_registry.h"

#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "maliput/api/rules/discrete_value_rule.h"
#include "maliput/api/rules/rule.h"

namespace maliput {
namespace test {
namespace {

// Holds the information to evaluate the rule type build by `builder` function.
struct BuildDiscreteValueRuleTypeExpectedValues {
  std::string type_id;
  std::vector<int> severities;
  std::vector<std::string> values;
  std::function<std::pair<api::rules::Rule::TypeId, std::vector<api::rules::DiscreteValueRule::DiscreteValue>>()>
      builder;
};

// Tests build rule type functions.
class BuildDiscreteValueRuleTypeTest : public ::testing::TestWithParam<BuildDiscreteValueRuleTypeExpectedValues> {
 protected:
  void SetUp() override { expectation_ = GetParam(); }

  bool HaveDiscreteValueWith(const std::vector<api::rules::DiscreteValueRule::DiscreteValue>& values, int severity,
                             const std::string& value) {
    const auto discrete_value = api::rules::MakeDiscreteValue(severity, {} /* related_rules */, value);
    return std::find(values.begin(), values.end(), discrete_value) != values.end();
  }

  BuildDiscreteValueRuleTypeExpectedValues expectation_;
};

std::vector<BuildDiscreteValueRuleTypeExpectedValues> BuildDiscreteValueRuleTypeTestParameters() {
  return {
      {"DirectionUsageRuleType",
       {api::rules::Rule::State::kStrict},
       {"WithS", "AgainstS", "Bidirectional", "BidirectionalTurnOnly", "NoUse", "Parking", "Undefined"},
       BuildDirectionUsageRuleType},
      {"RightOfWayRuleType",
       {api::rules::Rule::State::kStrict, api::rules::Rule::State::kBestEffort},
       {
           "Go",
           "Stop",
           "StopAndGo",
       },
       BuildRightOfWayRuleType},
      {"VehicleStopInZoneBehaviorRuleType",
       {api::rules::Rule::State::kStrict},
       {"DoNotStop", "5MinuteParking", "30MinuteParking", "45MinuteParking", "1HourParking", "2HourParking",
        "4HourParking", "UnconstrainedParking"},
       BuildVehicleStopInZoneBehaviorRuleType},
  };
}

TEST_P(BuildDiscreteValueRuleTypeTest, EvaluateRuleTypes) {
  const std::pair<api::rules::Rule::TypeId, std::vector<api::rules::DiscreteValueRule::DiscreteValue>> dut =
      expectation_.builder();

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
