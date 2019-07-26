/* clang-format off to disable clang-format-includes */
#include "maliput/api/rules/rule.h"
/* clang-format on */

#include <set>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "maliput/api/rules/regions.h"

namespace maliput {
namespace api {
namespace rules {
namespace {

// Evaluates RangeValueRule constructor and accessors.
GTEST_TEST(RangeValueRuleTest, ConstructorAndAccessors) {
  const Rule::Id kId("RuleId");
  const Rule::TypeId kTypeId("RuleTypeId");
  const SRange kSRange(10., 20.);
  const LaneId kLaneId("LaneId");
  const LaneSRoute kZone({LaneSRange(kLaneId, kSRange)});
  const std::vector<const Rule*> kRelatedRules{};
  const std::set<RangeValueRule::Range> kRanges{
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
  EXPECT_EQ(dut.related_rules().size(), 0.);
  EXPECT_EQ(dut.ranges().size(), kRanges.size());
  for (const RangeValueRule::Range& range : dut.ranges()) {
    EXPECT_NE(kRanges.find(range), kRanges.end());
  }
}

// Evaluates the less than operator overload for RangeValueRule::Range
GTEST_TEST(RangeTest, LessThanOperator) {
  const RangeValueRule::Range range_1{
      "range_description_1", 123. /* min */, 456. /* max */};
  const RangeValueRule::Range range_2{
      "range_description_2", 456. /* min */, 789. /* max */};
  const RangeValueRule::Range range_3{
      "range_description_3", 123. /* min */, 789. /* max */};
  const RangeValueRule::Range range_4{
      "range_description_4", 123. /* min */, 456. /* max */};

  // First, `min` is compared.
  EXPECT_TRUE(range_1 < range_2);
  // When `min` attributes are equal, then `max` is compared.
  EXPECT_TRUE(range_1 < range_3);
  // When `min` and `max` attributes are equal, then `description` is compared.
  EXPECT_TRUE(range_1 < range_4);
  // Self-comparison must be false.
  EXPECT_FALSE(range_1 < range_1);
}

// Evaluates RangeValueRule constructor and accessors.
GTEST_TEST(DiscreteValueRuleTest, ConstructorAndAccessors) {
  const Rule::Id kId("RuleId");
  const Rule::TypeId kTypeId("RuleTypeId");
  const SRange kSRange(10., 20.);
  const LaneId kLaneId("LaneId");
  const LaneSRoute kZone({LaneSRange(kLaneId, kSRange)});
  const std::vector<const Rule*> kRelatedRules{};
  const std::set<std::string> kDiscreteValues{
      "rule_state_value_1", "rule_state_value_2"};

  const DiscreteValueRule dut(
      kId, kTypeId, kZone, kRelatedRules, kDiscreteValues);

  EXPECT_EQ(dut.id(), kId);
  EXPECT_EQ(dut.type_id(), kTypeId);
  EXPECT_EQ(dut.zone().ranges().size(), 1);
  EXPECT_EQ(dut.zone().ranges()[0].lane_id(), kLaneId);
  EXPECT_EQ(dut.zone().ranges()[0].s_range().s0(), kSRange.s0());
  EXPECT_EQ(dut.zone().ranges()[0].s_range().s1(), kSRange.s1());
  EXPECT_EQ(dut.related_rules().size(), 0.);
  EXPECT_EQ(dut.value_states().size(), kDiscreteValues.size());
  for (const std::string& discrete_state_value : dut.value_states()) {
    EXPECT_NE(kDiscreteValues.find(discrete_state_value),
              kDiscreteValues.end());
  }
}


}  // namespace
}  // namespace rules
}  // namespace api
}  // namespace maliput
