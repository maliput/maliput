/* clang-format off to disable clang-format-includes */
#include "maliput/api/rules/rule.h"
/* clang-format on */

#include <gtest/gtest.h>

#include "maliput/common/assertion_error.h"

namespace maliput {
namespace api {
namespace rules {
namespace {

class RuleTest : public ::testing::Test {
 protected:
  const Rule::TypeId kTypeId{"RuleTypeIdA"};
  const Rule::Id kId{"RuleTypeIdA/RuleIdA"};
  const SRange kSRange{10., 20.};
  const LaneId kLaneId{"LaneId"};
  const LaneSRoute kZone{{LaneSRange(kLaneId, kSRange)}};
  const std::vector<Rule::Id> kRelatedRules{Rule::Id("RuleTypeIdB/RuleIdB"), Rule::Id("RuleTypeIdC/RuleIdC")};

};

// Evaluates RangeValueRule constructor.
TEST_F(RuleTest, RangeValueRuleConstructor) {
  const std::vector<RangeValueRule::Range> kRanges{
      {"range_description_1", 123. /* min */, 456. /* max */},
      {"range_description_2", 789. /* min */, 1234. /* max */},
  };

  EXPECT_NO_THROW(RangeValueRule(kId, kTypeId, kZone, kRelatedRules, kRanges));
  EXPECT_NO_THROW(RangeValueRule(kId, kTypeId, kZone, {} /* related rules */, kRanges));

  // Duplicated related rules.
  const std::vector<Rule::Id> kDuplicatedRelatedRules{Rule::Id("RuleTypeIdB/RuleIdB"), Rule::Id("RuleTypeIdB/RuleIdB")};
  EXPECT_THROW(RangeValueRule(kId, kTypeId, kZone, kDuplicatedRelatedRules, kRanges),
               maliput::common::assertion_error);
  // Empty ranges.
  EXPECT_THROW(RangeValueRule(kId, kTypeId, kZone, {} /* related rules */, {} /* ranges */),
               maliput::common::assertion_error);
  // Duplicated ranges.
  const std::vector<RangeValueRule::Range> kDuplicatedRanges{
      {"range_description_1", 123. /* min */, 456. /* max */},
      {"range_description_1", 123. /* min */, 456. /* max */},
  };
  EXPECT_THROW(RangeValueRule(kId, kTypeId, kZone, {} /* related rules */, kDuplicatedRanges),
               maliput::common::assertion_error);
  // RangeValueRule::Range::min is greater than RangeValueRule::Range::max.
  const std::vector<RangeValueRule::Range> kShiftedRanges{
      {"range_description_3", 456. /* min */, 123. /* max */}};
  EXPECT_THROW(RangeValueRule(kId, kTypeId, kZone, {} /* related rules */, kShiftedRanges),
               maliput::common::assertion_error);
}

// Evaluates RangeValueRule accessors.
TEST_F(RuleTest, RangeValueRuleAccessors) {
  const std::vector<RangeValueRule::Range> kRanges{
      {"range_description_1", 123. /* min */, 456. /* max */},
      {"range_description_2", 789. /* min */, 1234. /* max */},
  };

  const RangeValueRule dut(kId, kTypeId, kZone, kRelatedRules, kRanges);

  EXPECT_EQ(dut.id(), kId);
  EXPECT_EQ(dut.type_id(), kTypeId);
  EXPECT_EQ(dut.zone().ranges().size(), 1);
  EXPECT_EQ(dut.zone().ranges()[0].lane_id(), kLaneId);
  EXPECT_EQ(dut.zone().ranges()[0].s_range().s0(), kSRange.s0());
  EXPECT_EQ(dut.zone().ranges()[0].s_range().s1(), kSRange.s1());
  EXPECT_EQ(dut.related_rules().size(), kRelatedRules.size());
  for (const Rule::Id& related_rule : dut.related_rules()) {
    EXPECT_NE(std::find(kRelatedRules.begin(), kRelatedRules.end(), related_rule), kRelatedRules.end());
  }
  EXPECT_EQ(dut.ranges().size(), kRanges.size());
  for (const RangeValueRule::Range& range : dut.ranges()) {
    EXPECT_NE(std::find(kRanges.begin(), kRanges.end(), range), kRanges.end());
  }
}

// Evaluates the equal and not equal operator overloads for
// RangeValueRule::Range.
GTEST_TEST(RangeTest, EqualOperator) {
  const RangeValueRule::Range range_1{"range_description_1", 123. /* min */, 456. /* max */};
  const RangeValueRule::Range range_2{"range_description_1", 456. /* min */, 456. /* max */};
  const RangeValueRule::Range range_3{"range_description_1", 123. /* min */, 789. /* max */};
  const RangeValueRule::Range range_4{"range_description_4", 123. /* min */, 456. /* max */};


  EXPECT_TRUE(range_1 == range_1);
  EXPECT_FALSE(range_1 != range_1);
  // `min` is different.
  EXPECT_TRUE(range_1 != range_2);
  EXPECT_FALSE(range_1 == range_2);
  // `max` is different.
  EXPECT_TRUE(range_1 != range_3);
  EXPECT_FALSE(range_1 == range_3);
  // `description` is different.
  EXPECT_TRUE(range_1 != range_4);
  EXPECT_FALSE(range_1 == range_4);
}

// Evaluates DiscreteValueRule constructor.
TEST_F(RuleTest, DiscreteValueRuleConstructor) {
  const std::vector<std::string> kDiscreteValues{"rule_state_value_1", "rule_state_value_2"};

  EXPECT_NO_THROW(DiscreteValueRule(kId, kTypeId, kZone, {} /* related rules */, kDiscreteValues));
  EXPECT_NO_THROW(DiscreteValueRule(kId, kTypeId, kZone, kRelatedRules, kDiscreteValues));

  // Duplicated related rules.
  const std::vector<Rule::Id> kDuplicatedRelatedRules{Rule::Id("RuleTypeIdB/RuleIdB"), Rule::Id("RuleTypeIdB/RuleIdB")};
  EXPECT_THROW(DiscreteValueRule(kId, kTypeId, kZone, kDuplicatedRelatedRules, kDiscreteValues),
               maliput::common::assertion_error);
  // Empty discrete values.
  EXPECT_THROW(DiscreteValueRule(kId, kTypeId, kZone, kRelatedRules, {} /* discrete_values */),
               maliput::common::assertion_error);
  // Duplicated ranges.
  const std::vector<std::string> kDuplicatedDiscreteValues{"rule_state_value_1", "rule_state_value_1"};
  EXPECT_THROW(DiscreteValueRule(kId, kTypeId, kZone, kRelatedRules, kDuplicatedDiscreteValues),
               maliput::common::assertion_error);
}

// Evaluates DiscreteValueRule accessors.
TEST_F(RuleTest, DiscreteValueRuleAccessors) {
  const std::vector<std::string> kDiscreteValues{"rule_state_value_1", "rule_state_value_2"};

  const DiscreteValueRule dut(kId, kTypeId, kZone, kRelatedRules, kDiscreteValues);

  EXPECT_EQ(dut.id(), kId);
  EXPECT_EQ(dut.type_id(), kTypeId);
  EXPECT_EQ(dut.zone().ranges().size(), 1);
  EXPECT_EQ(dut.zone().ranges()[0].lane_id(), kLaneId);
  EXPECT_EQ(dut.zone().ranges()[0].s_range().s0(), kSRange.s0());
  EXPECT_EQ(dut.zone().ranges()[0].s_range().s1(), kSRange.s1());
  EXPECT_EQ(dut.related_rules().size(), kRelatedRules.size());
  for (const Rule::Id& related_rule : dut.related_rules()) {
    EXPECT_NE(std::find(kRelatedRules.begin(), kRelatedRules.end(), related_rule), kRelatedRules.end());
  }
  EXPECT_EQ(dut.values().size(), kDiscreteValues.size());
  for (const std::string& discrete_state_value : dut.values()) {
    EXPECT_NE(std::find(kDiscreteValues.begin(), kDiscreteValues.end(), discrete_state_value), kDiscreteValues.end());
  }
}

}  // namespace
}  // namespace rules
}  // namespace api
}  // namespace maliput
