/* clang-format off to disable clang-format-includes */
#include "maliput/api/rules/rule_registry.h"
/* clang-format on */

#include <map>
#include <set>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "maliput/api/rules/rule.h"
#include "maliput/common/assertion_error.h"

namespace maliput {
namespace api {
namespace rules {
namespace test {

// Evaluates queries to an empty RuleRegistry.
GTEST_TEST(EmptyRuleRegistry, AccessorsTest) {
  const RuleRegistry dut;

  EXPECT_TRUE(dut.RangeValueRuleTypes().empty());
  EXPECT_TRUE(dut.DiscreteValueRuleTypes().empty());

  const RuleRegistry::QueryResult result = dut.FindRuleTypeBy(
      Rule::TypeId("any_rule_type"));
  EXPECT_FALSE(result.range_value_rule_type.has_value());
  EXPECT_FALSE(result.discrete_value_rule_type.has_value());
}

// Evaluates queries after registering range based rule types.
GTEST_TEST(RegisterRangeValueRule, RegisterAndQueryTest) {
  const Rule::TypeId kRangeValueRuleTypeA("RangeValueRuleTypeA");
  const Rule::TypeId kRangeValueRuleTypeB("RangeValueRuleTypeB");

  RuleRegistry dut;
  // Registers range value rule types.
  EXPECT_NO_THROW({ dut.RegisterRangeValueRule(kRangeValueRuleTypeA); });
  EXPECT_NO_THROW({ dut.RegisterRangeValueRule(kRangeValueRuleTypeB); });
  // Throws because of duplicated ID.
  EXPECT_THROW({ dut.RegisterRangeValueRule(kRangeValueRuleTypeB); },
               maliput::common::assertion_error);
  EXPECT_THROW({ dut.RegisterDiscreteValueRule(kRangeValueRuleTypeB,
                                               {"SomeValue"}); },
               maliput::common::assertion_error);

  EXPECT_TRUE(dut.DiscreteValueRuleTypes().empty());

  const std::set<Rule::TypeId> range_value_rule_types =
      dut.RangeValueRuleTypes();
  EXPECT_EQ(range_value_rule_types.size(), 2);
  for (const Rule::TypeId& rule_type :
       { kRangeValueRuleTypeA, kRangeValueRuleTypeB}) {
    EXPECT_NE(range_value_rule_types.find(rule_type),
              range_value_rule_types.end());
  }

  // Finds each type.
  RuleRegistry::QueryResult result = dut.FindRuleTypeBy(kRangeValueRuleTypeA);
  EXPECT_TRUE(result.range_value_rule_type.has_value());
  EXPECT_EQ(result.range_value_rule_type.value(), kRangeValueRuleTypeA);
  EXPECT_FALSE(result.discrete_value_rule_type.has_value());

  result = dut.FindRuleTypeBy(kRangeValueRuleTypeB);
  EXPECT_TRUE(result.range_value_rule_type.has_value());
  EXPECT_EQ(result.range_value_rule_type.value(), kRangeValueRuleTypeB);
  EXPECT_FALSE(result.discrete_value_rule_type.has_value());

  result = dut.FindRuleTypeBy(Rule::TypeId("any_rule_type"));
  EXPECT_FALSE(result.range_value_rule_type.has_value());
  EXPECT_FALSE(result.discrete_value_rule_type.has_value());
}

// Evaluates queries after registering discrete value based rule types.
GTEST_TEST(RegisterDiscreteValueRule, RegisterAndQueryTest) {
  const Rule::TypeId kDiscreteValueRuleTypeA("DiscreteValueTypeA");
  const std::set<std::string> kDiscreteValuesA{"ValueA1", "ValueA2"};
  const Rule::TypeId kDiscreteValueRuleTypeB("RangeValueRuleTypeB");
  const std::set<std::string> kDiscreteValuesB{"ValueB1", "ValueB2", "ValueB3"};

  RuleRegistry dut;
  // Registers range value rule types.
  EXPECT_NO_THROW({ dut.RegisterDiscreteValueRule(kDiscreteValueRuleTypeA,
                                                  kDiscreteValuesA); });
  EXPECT_NO_THROW({ dut.RegisterDiscreteValueRule(kDiscreteValueRuleTypeB,
                                                  kDiscreteValuesB); });
  // Throws because of duplicated ID.
  EXPECT_THROW({ dut.RegisterDiscreteValueRule(kDiscreteValueRuleTypeB,
                                               {"SomeValue"}); },
               maliput::common::assertion_error);
  EXPECT_THROW({ dut.RegisterRangeValueRule(kDiscreteValueRuleTypeB); },
               maliput::common::assertion_error);
  // Throws because of empty set.
  EXPECT_THROW({ dut.RegisterDiscreteValueRule(Rule::TypeId("SomeRuleType"),
                                               {}); },
               maliput::common::assertion_error);

  EXPECT_TRUE(dut.RangeValueRuleTypes().empty());

  const std::map<Rule::TypeId, std::set<std::string>>
      discrete_value_rule_types = dut.DiscreteValueRuleTypes();
  EXPECT_EQ(discrete_value_rule_types.size(), 2);
  for (const auto& rule_values :
       std::map<Rule::TypeId, std::set<std::string>>{
           {kDiscreteValueRuleTypeA, kDiscreteValuesA},
           {kDiscreteValueRuleTypeB, kDiscreteValuesB}}) {
    const auto found_rule_values =
        discrete_value_rule_types.find(rule_values.first);
    EXPECT_NE(found_rule_values, discrete_value_rule_types.end());
    EXPECT_EQ(found_rule_values->second.size(), rule_values.second.size());
    for (const std::string& value : found_rule_values->second) {
      EXPECT_NE(rule_values.second.find(value), rule_values.second.end());
    }
  }

  // Finds each type.
  RuleRegistry::QueryResult result =
      dut.FindRuleTypeBy(kDiscreteValueRuleTypeA);
  EXPECT_FALSE(result.range_value_rule_type.has_value());
  EXPECT_TRUE(result.discrete_value_rule_type.has_value());
  EXPECT_EQ(result.discrete_value_rule_type->first, kDiscreteValueRuleTypeA);
  EXPECT_EQ(result.discrete_value_rule_type->second.size(),
            kDiscreteValuesA.size());
  for (const std::string& value : result.discrete_value_rule_type->second) {
    EXPECT_NE(kDiscreteValuesA.find(value), kDiscreteValuesA.end());
  }


  result = dut.FindRuleTypeBy(kDiscreteValueRuleTypeB);
  EXPECT_FALSE(result.range_value_rule_type.has_value());
  EXPECT_TRUE(result.discrete_value_rule_type.has_value());
  EXPECT_EQ(result.discrete_value_rule_type->first, kDiscreteValueRuleTypeB);
  EXPECT_EQ(result.discrete_value_rule_type->second.size(),
            kDiscreteValuesB.size());
  for (const std::string& value : result.discrete_value_rule_type->second) {
    EXPECT_NE(kDiscreteValuesB.find(value), kDiscreteValuesB.end());
  }


  result = dut.FindRuleTypeBy(Rule::TypeId("any_rule_type"));
  EXPECT_FALSE(result.range_value_rule_type.has_value());
  EXPECT_FALSE(result.discrete_value_rule_type.has_value());
}

// Registers range value based rules and discrete value based rules, then builds
// rules.
GTEST_TEST(RegisterAndBuildTest, RegisterAndBuild) {
  const Rule::TypeId kRangeValueRuleType("RangeValueRuleType");
  const Rule::Id kRangeRuleId("RangeValueRuleType/RangeRuleId");
  const LaneSRoute kZone({LaneSRange(LaneId("LaneId"), SRange(10., 20.))});
  const RangeValueRule::Range kRange{"range_description", 123. /* min */,
                                     456. /* max */};

  const Rule::TypeId kDiscreteValueRuleType("DiscreteValueType");
  const Rule::Id kDiscreteValueRuleId("DiscreteValueType/DiscreteValueRuleId");
  const std::set<std::string> kDiscreteValues{"Value1", "Value2", "Value3"};

  const Rule::TypeId kUnregisteredRuleType("UnregisteredRuleType");

  RuleRegistry dut;

  dut.RegisterRangeValueRule(kRangeValueRuleType);
  dut.RegisterDiscreteValueRule(kDiscreteValueRuleType, kDiscreteValues);

  // Builds and evaluates a range value based rule.
  const RangeValueRule range_value_rule = dut.BuildRangeValueRule(
      kRangeRuleId, kRangeValueRuleType, kZone, {} /* related rules */,
      {kRange});
  EXPECT_EQ(range_value_rule.id(), kRangeRuleId);
  EXPECT_EQ(range_value_rule.type_id(), kRangeValueRuleType);
  EXPECT_EQ(range_value_rule.zone().ranges().size(), 1);
  EXPECT_EQ(range_value_rule.zone().ranges()[0].lane_id(),
            kZone.ranges()[0].lane_id());
  EXPECT_EQ(range_value_rule.zone().ranges()[0].s_range().s0(),
            kZone.ranges()[0].s_range().s0());
  EXPECT_EQ(range_value_rule.zone().ranges()[0].s_range().s1(),
            kZone.ranges()[0].s_range().s1());
  EXPECT_EQ(range_value_rule.related_rules().size(), 0.);
  EXPECT_EQ(range_value_rule.ranges().size(), 1);
  EXPECT_EQ(range_value_rule.ranges().begin()->description, kRange.description);
  EXPECT_EQ(range_value_rule.ranges().begin()->min, kRange.min);
  EXPECT_EQ(range_value_rule.ranges().begin()->max, kRange.max);

  // Unregistered type.
  EXPECT_THROW({
      dut.BuildRangeValueRule(Rule::Id("RuleId"), kUnregisteredRuleType, kZone,
                              {} /* related rules */, {kRange}); },
      maliput::common::assertion_error);

  // Builds and evaluates a discrete value based rule.
  const DiscreteValueRule discrete_value_rule = dut.BuildDiscreteValueRule(
      kDiscreteValueRuleId, kDiscreteValueRuleType, kZone,
      {} /* related rules */, {"Value1", "Value3"});
  EXPECT_EQ(discrete_value_rule.id(), kDiscreteValueRuleId);
  EXPECT_EQ(discrete_value_rule.type_id(), kDiscreteValueRuleType);
  EXPECT_EQ(discrete_value_rule.zone().ranges().size(), 1);
  EXPECT_EQ(discrete_value_rule.zone().ranges()[0].lane_id(),
            kZone.ranges()[0].lane_id());
  EXPECT_EQ(discrete_value_rule.zone().ranges()[0].s_range().s0(),
            kZone.ranges()[0].s_range().s0());
  EXPECT_EQ(discrete_value_rule.zone().ranges()[0].s_range().s1(),
            kZone.ranges()[0].s_range().s1());
  EXPECT_EQ(discrete_value_rule.related_rules().size(), 0.);
  EXPECT_EQ(discrete_value_rule.related_rules().size(), 0.);
  EXPECT_EQ(discrete_value_rule.value_states().size(), 2);
  for (const std::string& discrete_state_value : {"Value1", "Value3"}) {
    EXPECT_NE(discrete_value_rule.value_states().find(discrete_state_value),
              discrete_value_rule.value_states().end());
  }
  // Unregistered type.
  EXPECT_THROW({
      dut.BuildDiscreteValueRule(Rule::Id("RuleId"), kUnregisteredRuleType,
                                 kZone, {} /* related rules */,
                                 {"Value1", "Value3"}); },
      maliput::common::assertion_error);
  // Unregistered discrete value for the type.
  EXPECT_THROW({
      dut.BuildDiscreteValueRule(kDiscreteValueRuleId, kDiscreteValueRuleType,
                                 kZone, {} /* related rules */,
                                 {"Value1", "Value4"}); },
      maliput::common::assertion_error);
}



}  // namespace test
}  // namespace rules
}  // namespace api
}  // namespace maliput
