#include "maliput/base/manual_rulebook.h"

#include <gtest/gtest.h>

#include "maliput/api/rules/regions.h"
#include "maliput/api/rules/right_of_way_rule.h"
#include "maliput/api/rules/speed_limit_rule.h"
#include "maliput/common/assertion_error.h"
#include "maliput/test_utilities/rules_compare.h"
#include "maliput/test_utilities/rules_direction_usage_compare.h"
#include "maliput/test_utilities/rules_right_of_way_compare.h"
#include "maliput/test_utilities/rules_speed_limit_compare.h"
#include "maliput/test_utilities/rules_test_utilities.h"

namespace maliput {
namespace {

using api::LaneId;
using api::rules::DirectionUsageRule;
using api::rules::DiscreteValueRule;
using api::rules::LaneSRange;
using api::rules::LaneSRoute;
using api::rules::MakeDiscreteValue;
using api::rules::MakeRange;
using api::rules::RangeValueRule;
using api::rules::RightOfWayRule;
using api::rules::RoadRulebook;
using api::rules::Rule;
using api::rules::SpeedLimitRule;

class ManualRulebookTest : public ::testing::Test {
 protected:
  const LaneSRange kZone{LaneId("a"), {10., 20.}};

  const RightOfWayRule kRightOfWay{
      RightOfWayRule::Id("rowr_id"),
      LaneSRoute({kZone}),
      RightOfWayRule::ZoneType::kStopExcluded,
      {RightOfWayRule::State{RightOfWayRule::State::Id("rowr_state_id"), RightOfWayRule::State::Type::kStopThenGo, {}}},
      {} /* related_bulb_groups */};

  const SpeedLimitRule kSpeedLimit{SpeedLimitRule::Id("slr_id"), kZone, SpeedLimitRule::Severity::kStrict, 0., 44.};

  const DirectionUsageRule kDirectionUsage{
      DirectionUsageRule::Id("dur_id"),
      kZone,
      {DirectionUsageRule::State(DirectionUsageRule::State::Id("dur_state"), DirectionUsageRule::State::Type::kWithS,
                                 DirectionUsageRule::State::Severity::kStrict)}};

  const Rule::Id kDiscreteValueRuleId{"dvrt/dvr_id"};

  const DiscreteValueRule kDiscreteValueRule{
      kDiscreteValueRuleId,
      Rule::TypeId("dvrt"),
      LaneSRoute({kZone}),
      {} /* related rules */,
      {MakeDiscreteValue(Rule::State::kStrict, "value1"), MakeDiscreteValue(Rule::State::kBestEffort, "value2")}};

  const Rule::Id kRangeValueRuleId{"rvrt/rvr"};

  const RangeValueRule kRangeValueRule{kRangeValueRuleId,
                                       Rule::TypeId("rvrt"),
                                       LaneSRoute({kZone}),
                                       {} /* related_rules */,
                                       {MakeRange(Rule::State::kStrict, "description", 123., 456.)}};
};

TEST_F(ManualRulebookTest, DefaultConstructor) { ManualRulebook dut; }

TEST_F(ManualRulebookTest, AddGetRemoveRightOfWay) {
  ManualRulebook dut;

  EXPECT_THROW(dut.GetRule(kRightOfWay.id()), std::out_of_range);
  dut.AddRule(kRightOfWay);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut.GetRule(kRightOfWay.id()), kRightOfWay));
  EXPECT_THROW(dut.AddRule(kRightOfWay), maliput::common::assertion_error);
  dut.RemoveRule(kRightOfWay.id());
  EXPECT_THROW(dut.GetRule(kRightOfWay.id()), std::out_of_range);
  EXPECT_THROW(dut.RemoveRule(kRightOfWay.id()), maliput::common::assertion_error);
}

TEST_F(ManualRulebookTest, AddGetRemoveSpeedLimit) {
  ManualRulebook dut;

  EXPECT_THROW(dut.GetRule(kSpeedLimit.id()), std::out_of_range);
  dut.AddRule(kSpeedLimit);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut.GetRule(kSpeedLimit.id()), kSpeedLimit));
  EXPECT_THROW(dut.AddRule(kSpeedLimit), maliput::common::assertion_error);
  dut.RemoveRule(kSpeedLimit.id());
  EXPECT_THROW(dut.GetRule(kSpeedLimit.id()), std::out_of_range);
  EXPECT_THROW(dut.RemoveRule(kSpeedLimit.id()), maliput::common::assertion_error);
}

TEST_F(ManualRulebookTest, AddGetRemoveDirectionUsage) {
  ManualRulebook dut;

  EXPECT_THROW(dut.GetRule(kDirectionUsage.id()), std::out_of_range);
  dut.AddRule(kDirectionUsage);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut.GetRule(kDirectionUsage.id()), kDirectionUsage));
  EXPECT_THROW(dut.AddRule(kDirectionUsage), maliput::common::assertion_error);
  dut.RemoveRule(kDirectionUsage.id());
  EXPECT_THROW(dut.GetRule(kDirectionUsage.id()), std::out_of_range);
  EXPECT_THROW(dut.RemoveRule(kDirectionUsage.id()), maliput::common::assertion_error);
}

TEST_F(ManualRulebookTest, AddGetRemoveRangeValueRule) {
  ManualRulebook dut;

  EXPECT_THROW(dut.GetRangeValueRule(kRangeValueRule.id()), std::out_of_range);
  dut.AddRule(kRangeValueRule);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut.GetRangeValueRule(kRangeValueRule.id()), kRangeValueRule));
  EXPECT_THROW(dut.AddRule(kRangeValueRule), maliput::common::assertion_error);
  dut.RemoveRule(kRangeValueRule.id());
  EXPECT_THROW(dut.GetRangeValueRule(kRangeValueRule.id()), std::out_of_range);
  EXPECT_THROW(dut.RemoveRule(kRangeValueRule.id()), maliput::common::assertion_error);

  const DiscreteValueRule kDiscreteValueRuleWithSameId{
      kRangeValueRuleId,
      Rule::TypeId("dvrt"),
      LaneSRoute({kZone}),
      {} /* related rules */,
      {MakeDiscreteValue(Rule::State::kStrict, "value1"), MakeDiscreteValue(Rule::State::kStrict, "value2")}};
  dut.AddRule(kDiscreteValueRuleWithSameId);
  EXPECT_THROW(dut.AddRule(kRangeValueRule), maliput::common::assertion_error);
}

TEST_F(ManualRulebookTest, AddGetRemoveDiscreteValueRule) {
  ManualRulebook dut;

  EXPECT_THROW(dut.GetDiscreteValueRule(kDiscreteValueRule.id()), std::out_of_range);
  dut.AddRule(kDiscreteValueRule);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut.GetDiscreteValueRule(kDiscreteValueRule.id()), kDiscreteValueRule));
  EXPECT_THROW(dut.AddRule(kDiscreteValueRule), maliput::common::assertion_error);
  dut.RemoveRule(kDiscreteValueRule.id());
  EXPECT_THROW({ dut.GetDiscreteValueRule(kDiscreteValueRule.id()); }, std::out_of_range);
  EXPECT_THROW(dut.RemoveRule(kDiscreteValueRule.id()), maliput::common::assertion_error);

  const RangeValueRule kRangeValueRuleWithSameId{kDiscreteValueRuleId,
                                                 Rule::TypeId("rvrt"),
                                                 LaneSRoute({kZone}),
                                                 {} /* related_rules */,
                                                 {MakeRange(Rule::State::kStrict, "description", 123., 456.)}};
  dut.AddRule(kRangeValueRuleWithSameId);
  EXPECT_THROW(dut.AddRule(kDiscreteValueRule), maliput::common::assertion_error);
}

TEST_F(ManualRulebookTest, RemoveAll) {
  ManualRulebook dut;
  dut.RemoveAll();  // I.e., should work on empty rulebook.
  dut.AddRule(kRightOfWay);
  dut.AddRule(kSpeedLimit);
  dut.AddRule(kDirectionUsage);
  dut.AddRule(kDiscreteValueRule);
  dut.AddRule(kRangeValueRule);
  dut.RemoveAll();
  EXPECT_THROW(dut.GetRule(kRightOfWay.id()), std::out_of_range);
  EXPECT_THROW(dut.RemoveRule(kRightOfWay.id()), maliput::common::assertion_error);
  EXPECT_THROW(dut.GetRule(kSpeedLimit.id()), std::out_of_range);
  EXPECT_THROW(dut.RemoveRule(kSpeedLimit.id()), maliput::common::assertion_error);
  EXPECT_THROW(dut.GetRule(kDirectionUsage.id()), std::out_of_range);
  EXPECT_THROW(dut.RemoveRule(kDirectionUsage.id()), maliput::common::assertion_error);
  EXPECT_THROW(dut.GetDiscreteValueRule(kDiscreteValueRule.id()), std::out_of_range);
  EXPECT_THROW(dut.RemoveRule(kDiscreteValueRule.id()), maliput::common::assertion_error);
  EXPECT_THROW(dut.GetRangeValueRule(kRangeValueRule.id()), std::out_of_range);
  EXPECT_THROW(dut.RemoveRule(kRangeValueRule.id()), maliput::common::assertion_error);

  // Since the original rules are gone, it should be possible to re-add them.
  dut.AddRule(kRightOfWay);
  dut.AddRule(kSpeedLimit);
  dut.AddRule(kDirectionUsage);
  dut.AddRule(kDiscreteValueRule);
  dut.AddRule(kRangeValueRule);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut.GetRule(kRightOfWay.id()), kRightOfWay));
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut.GetRule(kSpeedLimit.id()), kSpeedLimit));
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut.GetRule(kDirectionUsage.id()), kDirectionUsage));
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut.GetDiscreteValueRule(kDiscreteValueRule.id()), kDiscreteValueRule));
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut.GetRangeValueRule(kRangeValueRule.id()), kRangeValueRule));
}

TEST_F(ManualRulebookTest, FindRules) {
  ManualRulebook dut;
  dut.AddRule(kSpeedLimit);
  dut.AddRule(kRightOfWay);
  dut.AddRule(kDirectionUsage);
  dut.AddRule(kDiscreteValueRule);
  dut.AddRule(kRangeValueRule);

  const double kZeroTolerance = 0.;

  const RoadRulebook::QueryResults empty = dut.FindRules({}, kZeroTolerance);
  EXPECT_EQ(empty.right_of_way.size(), 0);
  EXPECT_EQ(empty.speed_limit.size(), 0);
  EXPECT_EQ(empty.direction_usage.size(), 0);
  EXPECT_EQ(empty.discrete_value_rules.size(), 0);
  EXPECT_EQ(empty.range_value_rules.size(), 0);

  const RoadRulebook::QueryResults nonempty = dut.FindRules({kZone}, kZeroTolerance);
  EXPECT_EQ(nonempty.right_of_way.size(), 1);
  EXPECT_EQ(nonempty.speed_limit.size(), 1);
  EXPECT_EQ(nonempty.direction_usage.size(), 1);
  EXPECT_EQ(nonempty.discrete_value_rules.size(), 1);
  EXPECT_EQ(nonempty.range_value_rules.size(), 1);

  const LaneSRange kReversedZone(kZone.lane_id(), {kZone.s_range().s1(), kZone.s_range().s0()});
  const RoadRulebook::QueryResults reversed = dut.FindRules({kReversedZone}, kZeroTolerance);
  EXPECT_EQ(reversed.right_of_way.size(), 1);
  EXPECT_EQ(reversed.speed_limit.size(), 1);
  EXPECT_EQ(reversed.direction_usage.size(), 1);
  EXPECT_EQ(reversed.discrete_value_rules.size(), 1);
  EXPECT_EQ(reversed.range_value_rules.size(), 1);

  const double kNonzeroTolerance = 0.1;

  ASSERT_LT(kZone.s_range().s0(), kZone.s_range().s1());
  // Construct a range that just barely overlaps the kNonzeroTolerance band
  // of kZone.s1.
  const LaneSRange kNearbyRange(kZone.lane_id(),
                                {kZone.s_range().s1() + (0.9 * kNonzeroTolerance), kZone.s_range().s1() + 50.});
  const RoadRulebook::QueryResults nearby = dut.FindRules({kNearbyRange}, kNonzeroTolerance);
  EXPECT_EQ(nearby.right_of_way.size(), 1);
  EXPECT_EQ(nearby.speed_limit.size(), 1);
  EXPECT_EQ(nearby.direction_usage.size(), 1);
  EXPECT_EQ(nearby.discrete_value_rules.size(), 1);
  EXPECT_EQ(nearby.range_value_rules.size(), 1);

  // Construct a range that sits just outside of the kNonzeroTolerance band
  // of kZone.s1.
  const LaneSRange kTooFarRange(kZone.lane_id(),
                                {kZone.s_range().s1() + (1.1 * kNonzeroTolerance), kZone.s_range().s1() + 50.});
  const RoadRulebook::QueryResults toofar = dut.FindRules({kTooFarRange}, kNonzeroTolerance);
  EXPECT_EQ(toofar.right_of_way.size(), 0);
  EXPECT_EQ(toofar.speed_limit.size(), 0);
  EXPECT_EQ(toofar.direction_usage.size(), 0);
  EXPECT_EQ(toofar.discrete_value_rules.size(), 0);
  EXPECT_EQ(toofar.range_value_rules.size(), 0);
}

TEST_F(ManualRulebookTest, GetAllRules) {
  ManualRulebook dut;

  RoadRulebook::QueryResults result = dut.Rules();
  EXPECT_TRUE(result.right_of_way.empty());
  EXPECT_TRUE(result.speed_limit.empty());
  EXPECT_TRUE(result.direction_usage.empty());
  EXPECT_TRUE(result.discrete_value_rules.empty());
  EXPECT_TRUE(result.range_value_rules.empty());

  dut.AddRule(kSpeedLimit);
  dut.AddRule(kRightOfWay);
  result = dut.Rules();
  EXPECT_EQ(result.right_of_way.size(), 1);
  EXPECT_EQ(result.speed_limit.size(), 1);
  EXPECT_EQ(result.direction_usage.size(), 0);
  EXPECT_EQ(result.discrete_value_rules.size(), 0);
  EXPECT_EQ(result.range_value_rules.size(), 0);

  dut.AddRule(kDirectionUsage);
  result = dut.Rules();
  EXPECT_EQ(result.right_of_way.size(), 1);
  EXPECT_EQ(result.speed_limit.size(), 1);
  EXPECT_EQ(result.direction_usage.size(), 1);
  EXPECT_EQ(result.discrete_value_rules.size(), 0);
  EXPECT_EQ(result.range_value_rules.size(), 0);

  dut.AddRule(kDiscreteValueRule);
  result = dut.Rules();
  EXPECT_EQ(result.right_of_way.size(), 1);
  EXPECT_EQ(result.speed_limit.size(), 1);
  EXPECT_EQ(result.direction_usage.size(), 1);
  EXPECT_EQ(result.discrete_value_rules.size(), 1);
  EXPECT_EQ(result.range_value_rules.size(), 0);

  dut.AddRule(kRangeValueRule);
  result = dut.Rules();
  EXPECT_EQ(result.right_of_way.size(), 1);
  EXPECT_EQ(result.speed_limit.size(), 1);
  EXPECT_EQ(result.direction_usage.size(), 1);
  EXPECT_EQ(result.discrete_value_rules.size(), 1);
  EXPECT_EQ(result.range_value_rules.size(), 1);
}

}  // namespace
}  // namespace maliput
