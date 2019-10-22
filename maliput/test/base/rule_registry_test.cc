#include <gtest/gtest.h>

#include "maliput/api/rules/discrete_value_rule.h"
#include "maliput/base/rule_registry.h"

namespace maliput {
namespace test {
namespace {

GTEST_TEST(BuildDirectionUsageRuleTypeTest, BasicTest) {
  const std::pair<api::rules::Rule::TypeId, std::vector<api::rules::DiscreteValueRule::DiscreteValue>> dut =
      BuildDirectionUsageRuleType();
  const api::rules::Rule::RelatedRules empty_related_rules;
  const std::vector<api::rules::DiscreteValueRule::DiscreteValue> expected_values{
      api::rules::MakeDiscreteValue(api::rules::Rule::State::kStrict, empty_related_rules, "WithS"),
      api::rules::MakeDiscreteValue(api::rules::Rule::State::kStrict, empty_related_rules, "AgainstS"),
      api::rules::MakeDiscreteValue(api::rules::Rule::State::kStrict, empty_related_rules, "Bidirectional"),
      api::rules::MakeDiscreteValue(api::rules::Rule::State::kStrict, empty_related_rules, "BidirectionalTurnOnly"),
      api::rules::MakeDiscreteValue(api::rules::Rule::State::kStrict, empty_related_rules, "NoUse"),
      api::rules::MakeDiscreteValue(api::rules::Rule::State::kStrict, empty_related_rules, "Parking"),
      api::rules::MakeDiscreteValue(api::rules::Rule::State::kStrict, empty_related_rules, "Undefined")};

  EXPECT_EQ(dut.first, api::rules::Rule::TypeId("DirectionUsageRuleType"));
  for (const auto& expected_value : expected_values) {
    EXPECT_NE(std::find(dut.second.begin(), dut.second.end(), expected_value), dut.second.end());
  }
}

GTEST_TEST(DirectionUsageRuleTypeIdTest, BasicTest) {
  const maliput::api::rules::Rule::TypeId dut{DirectionUsageRuleTypeId()};
  EXPECT_EQ(dut, api::rules::Rule::TypeId("DirectionUsageRuleType"));
}

}  // namespace
}  // namespace test
}  // namespace maliput
