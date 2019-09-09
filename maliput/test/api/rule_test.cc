#include "maliput/api/rules/discrete_value_rule.h"
#include "maliput/api/rules/range_value_rule.h"

#include <gtest/gtest.h>

#include "maliput/common/assertion_error.h"
#include "maliput/test_utilities/mock.h"
#include "maliput/test_utilities/regions_test_utilities.h"
#include "maliput/test_utilities/rules_compare.h"
#include "maliput/test_utilities/rules_test_utilities.h"

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
  const Rule::RelatedRules kRelatedRules = api::test::CreateNonEmptyRelatedRules();
};

// Evaluates RangeValueRule constructor.
TEST_F(RuleTest, RangeValueRuleConstructor) {
  const std::vector<RangeValueRule::Range> kRanges{
      MakeRange(Rule::State::kStrict, api::test::CreateEmptyRelatedRules(), "range_description_1", 123. /* min */,
                456. /* max */),
      MakeRange(Rule::State::kBestEffort, api::test::CreateEmptyRelatedRules(), "range_description_2", 789. /* min */,
                1234. /* max */),
  };

  EXPECT_NO_THROW(RangeValueRule(kId, kTypeId, kZone, kRanges));

  // Duplicated Rule::Ids in RelatedRules in RangeValueRule::Range.
  const Rule::RelatedRules kDuplicatedRelatedRules{
      {"RuleGroup", {Rule::Id("RuleTypeIdB/RuleIdB"), Rule::Id("RuleTypeIdB/RuleIdB")}}};
  const RangeValueRule::Range kRangeWithDuplicatedRules =
      MakeRange(Rule::State::kStrict, kDuplicatedRelatedRules, "range_description_1", 123. /* min */, 456. /* max */);
  EXPECT_THROW(RangeValueRule(kId, kTypeId, kZone, {kRangeWithDuplicatedRules}), maliput::common::assertion_error);

  // Missing Rule::Ids in RelatedRules in RangeValueRule::Range.
  const Rule::RelatedRules kMissingRelatedRules{{"RuleGroup", {}}};
  const RangeValueRule::Range kRangeWithMissingRelatedRules =
      MakeRange(Rule::State::kStrict, kMissingRelatedRules, "range_description_1", 123. /* min */, 456. /* max */);
  EXPECT_THROW(RangeValueRule(kId, kTypeId, kZone, {kRangeWithMissingRelatedRules}), maliput::common::assertion_error);

  // Empty std::string for semantic group key in RelatedRules in RangeValueRule::Range.
  const Rule::RelatedRules kEmptyKeyRelatedRules{{"", {Rule::Id("RuleTypeIdB/RuleIdB")}}};
  const RangeValueRule::Range kRangeWithEmptyKeyRelatedRules =
      MakeRange(Rule::State::kStrict, kEmptyKeyRelatedRules, "range_description_1", 123. /* min */, 456. /* max */);
  EXPECT_THROW(RangeValueRule(kId, kTypeId, kZone, {kRangeWithEmptyKeyRelatedRules}), maliput::common::assertion_error);

  // Negative severity.
  const int kSeverityInvalid{-1};
  EXPECT_THROW(RangeValueRule(kId, kTypeId, kZone,
                              {MakeRange(kSeverityInvalid, api::test::CreateNonEmptyRelatedRules(),
                                         "range_description_1", 123. /* min */, 456. /* max */)}),
               maliput::common::assertion_error);
  // Empty ranges.
  EXPECT_THROW(RangeValueRule(kId, kTypeId, kZone, {} /* ranges */), maliput::common::assertion_error);
  // Duplicated ranges.
  const std::vector<RangeValueRule::Range> kDuplicatedRanges{
      MakeRange(Rule::State::kStrict, api::test::CreateEmptyRelatedRules(), "range_description_1", 123. /* min */,
                456. /* max */),
      MakeRange(Rule::State::kStrict, api::test::CreateEmptyRelatedRules(), "range_description_1", 123. /* min */,
                456. /* max */),
  };
  EXPECT_THROW(RangeValueRule(kId, kTypeId, kZone, kDuplicatedRanges), maliput::common::assertion_error);

  // RangeValueRule::Range::min is greater than RangeValueRule::Range::max.
  const std::vector<RangeValueRule::Range> kShiftedRanges{
      MakeRange(Rule::State::kStrict, api::test::CreateEmptyRelatedRules(), "range_description_3", 456. /* min */,
                123. /* max */)};
  EXPECT_THROW(RangeValueRule(kId, kTypeId, kZone, {kShiftedRanges}), maliput::common::assertion_error);
}

// Evaluates RangeValueRule accessors.
TEST_F(RuleTest, RangeValueRuleAccessors) {
  const std::vector<RangeValueRule::Range> kRanges{
      MakeRange(Rule::State::kStrict, api::test::CreateEmptyRelatedRules(), "range_description_1", 123. /* min */,
                456. /* max */),
      MakeRange(Rule::State::kBestEffort, api::test::CreateNonEmptyRelatedRules(), "range_description_2",
                789. /* min */, 1234. /* max */),
  };

  const RangeValueRule dut(kId, kTypeId, kZone, kRanges);

  EXPECT_EQ(dut.id(), kId);
  EXPECT_EQ(dut.type_id(), kTypeId);
  EXPECT_TRUE(MALIPUT_REGIONS_IS_EQUAL(dut.zone(), kZone));
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut.ranges(), kRanges));
}

// Evaluates the equal and not equal operator overloads for
// RangeValueRule::Range.
GTEST_TEST(RangeTest, EqualOperator) {
  const RangeValueRule::Range range_1 = MakeRange(Rule::State::kStrict, api::test::CreateEmptyRelatedRules(),
                                                  "range_description_1", 123. /* min */, 456. /* max */);
  const RangeValueRule::Range range_2 = MakeRange(Rule::State::kStrict, api::test::CreateEmptyRelatedRules(),
                                                  "range_description_1", 456. /* min */, 456. /* max */);
  const RangeValueRule::Range range_3 = MakeRange(Rule::State::kStrict, api::test::CreateEmptyRelatedRules(),
                                                  "range_description_1", 123. /* min */, 789. /* max */);
  const RangeValueRule::Range range_4 = MakeRange(Rule::State::kStrict, api::test::CreateEmptyRelatedRules(),
                                                  "range_description_4", 123. /* min */, 456. /* max */);
  const RangeValueRule::Range range_5 = MakeRange(Rule::State::kBestEffort, api::test::CreateEmptyRelatedRules(),
                                                  "range_description_1", 123. /* min */, 456. /* max */);
  const RangeValueRule::Range range_6 = MakeRange(Rule::State::kStrict, api::test::CreateNonEmptyRelatedRules(),
                                                  "range_description_6", 123. /* min */, 456. /* max */);

  // `related_rules` are empty.
  EXPECT_TRUE(range_1 == range_1);
  EXPECT_FALSE(range_1 != range_1);
  // `related_rules` are not empty.
  EXPECT_TRUE(range_6 == range_6);
  EXPECT_FALSE(range_6 != range_6);
  // `min` is different.
  EXPECT_TRUE(range_1 != range_2);
  EXPECT_FALSE(range_1 == range_2);
  // `max` is different.
  EXPECT_TRUE(range_1 != range_3);
  EXPECT_FALSE(range_1 == range_3);
  // `description` is different.
  EXPECT_TRUE(range_1 != range_4);
  EXPECT_FALSE(range_1 == range_4);
  // `severity` is different.
  EXPECT_TRUE(range_1 != range_5);
  EXPECT_FALSE(range_1 == range_5);
}

// Evaluates DiscreteValueRule constructor.
TEST_F(RuleTest, DiscreteValueRuleConstructor) {
  const std::vector<DiscreteValueRule::DiscreteValue> kDiscreteValues{
      MakeDiscreteValue(Rule::State::kStrict, api::test::CreateEmptyRelatedRules(), "rule_state_value_1"),
      MakeDiscreteValue(Rule::State::kBestEffort, api::test::CreateNonEmptyRelatedRules(), "rule_state_value_2")};

  EXPECT_NO_THROW(DiscreteValueRule(kId, kTypeId, kZone, kDiscreteValues));

  // Negative severity.
  const int kSeverityInvalid{-1};
  EXPECT_THROW(DiscreteValueRule(
                   kId, kTypeId, kZone,
                   {MakeDiscreteValue(kSeverityInvalid, api::test::CreateEmptyRelatedRules(), "rule_state_value")}),
               maliput::common::assertion_error);
  // Duplicated Rule::Ids in RelatedRules in DiscreteValueRule::DiscreteValue.
  const Rule::RelatedRules kDuplicatedRelatedRules{
      {"RuleGroup", {Rule::Id("RuleTypeIdB/RuleIdB"), Rule::Id("RuleTypeIdB/RuleIdB")}}};
  const DiscreteValueRule::DiscreteValue kDiscreteValueWithDuplicatedRules =
      MakeDiscreteValue(Rule::State::kStrict, kDuplicatedRelatedRules, "rule_state_value");
  EXPECT_THROW(DiscreteValueRule(kId, kTypeId, kZone, {kDiscreteValueWithDuplicatedRules}),
               maliput::common::assertion_error);
  // Missing Rule::Ids in RelatedRules in DiscreteValueRule::DiscreteValue.
  const Rule::RelatedRules kMissingRelatedRules{{"RuleGroup", {}}};
  const DiscreteValueRule::DiscreteValue kDiscreteValueWithMissingRelatedRules =
      MakeDiscreteValue(Rule::State::kStrict, kMissingRelatedRules, "rule_state_value");
  EXPECT_THROW(DiscreteValueRule(kId, kTypeId, kZone, {kDiscreteValueWithMissingRelatedRules}),
               maliput::common::assertion_error);
  // Duplicated ranges.
  const std::vector<DiscreteValueRule::DiscreteValue> kDuplicatedDiscreteValues{
      MakeDiscreteValue(Rule::State::kStrict, api::test::CreateNonEmptyRelatedRules(), "rule_state_value_1"),
      MakeDiscreteValue(Rule::State::kStrict, api::test::CreateNonEmptyRelatedRules(), "rule_state_value_1")};
  EXPECT_THROW(DiscreteValueRule(kId, kTypeId, kZone, kDuplicatedDiscreteValues), maliput::common::assertion_error);
  // Empty std::string for semantic group key in RelatedRules in DiscreteValueRule::DiscreteValue.
  const Rule::RelatedRules kEmptyKeyRelatedRules{{"", {Rule::Id("RuleTypeIdB/RuleIdB")}}};
  const DiscreteValueRule::DiscreteValue kDiscreteValueWithEmptyKeyRelatedRules =
      MakeDiscreteValue(Rule::State::kStrict, kEmptyKeyRelatedRules, "rule_state_value");
  EXPECT_THROW(DiscreteValueRule(kId, kTypeId, kZone, {kDiscreteValueWithEmptyKeyRelatedRules}),
               maliput::common::assertion_error);

  // Empty discrete values.
  EXPECT_THROW(DiscreteValueRule(kId, kTypeId, kZone, {} /* discrete_values */), maliput::common::assertion_error);
  // Duplicated discrete values.
  EXPECT_THROW(DiscreteValueRule(kId, kTypeId, kZone, kDuplicatedDiscreteValues), maliput::common::assertion_error);
}

// Evaluates DiscreteValueRule accessors.
TEST_F(RuleTest, DiscreteValueRuleAccessors) {
  const std::vector<DiscreteValueRule::DiscreteValue> kDiscreteValues{
      MakeDiscreteValue(Rule::State::kStrict, api::test::CreateEmptyRelatedRules(), "rule_state_value_1"),
      MakeDiscreteValue(Rule::State::kBestEffort, api::test::CreateNonEmptyRelatedRules(), "rule_state_value_2")};

  const DiscreteValueRule dut(kId, kTypeId, kZone, kDiscreteValues);

  EXPECT_EQ(dut.id(), kId);
  EXPECT_EQ(dut.type_id(), kTypeId);
  EXPECT_TRUE(MALIPUT_REGIONS_IS_EQUAL(dut.zone(), kZone));
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut.values(), kDiscreteValues));
}

// Evaluates the equal and not equal operator overloads for
// DiscreteValueRule::DiscreteValue.
GTEST_TEST(DiscreteValueTest, EqualOperator) {
  const DiscreteValueRule::DiscreteValue discrete_value_1 =
      MakeDiscreteValue(Rule::State::kStrict, api::test::CreateEmptyRelatedRules(), "value_1");
  const DiscreteValueRule::DiscreteValue discrete_value_2 =
      MakeDiscreteValue(Rule::State::kStrict, api::test::CreateEmptyRelatedRules(), "value_2");
  const DiscreteValueRule::DiscreteValue discrete_value_3 =
      MakeDiscreteValue(Rule::State::kBestEffort, api::test::CreateEmptyRelatedRules(), "value_1");
  const DiscreteValueRule::DiscreteValue discrete_value_4 =
      MakeDiscreteValue(Rule::State::kBestEffort, api::test::CreateNonEmptyRelatedRules(), "value_4");

  // `related_rules` are empty.
  EXPECT_TRUE(discrete_value_1 == discrete_value_1);
  EXPECT_FALSE(discrete_value_1 != discrete_value_1);
  // `value` is different.
  EXPECT_TRUE(discrete_value_1 != discrete_value_2);
  EXPECT_FALSE(discrete_value_1 == discrete_value_2);
  // `severity` is different.
  EXPECT_TRUE(discrete_value_1 != discrete_value_3);
  EXPECT_FALSE(discrete_value_1 == discrete_value_3);
  // `related_rules` are not empty.
  EXPECT_TRUE(discrete_value_4 == discrete_value_4);
  EXPECT_FALSE(discrete_value_4 != discrete_value_4);
}

}  // namespace
}  // namespace rules
}  // namespace api
}  // namespace maliput
