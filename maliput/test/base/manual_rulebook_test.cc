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
#include "maliput/base/manual_rulebook.h"

#include <gtest/gtest.h>

#include "maliput/api/regions.h"
#include "maliput/api/rules/right_of_way_rule.h"
#include "maliput/api/rules/speed_limit_rule.h"
#include "maliput/common/assertion_error.h"
#include "maliput/test_utilities/mock.h"
#include "maliput/test_utilities/rules_compare.h"
#include "maliput/test_utilities/rules_direction_usage_compare.h"
#include "maliput/test_utilities/rules_right_of_way_compare.h"
#include "maliput/test_utilities/rules_speed_limit_compare.h"
#include "maliput/test_utilities/rules_test_utilities.h"

namespace maliput {
namespace test {
namespace {

using api::LaneId;
using api::LaneSRange;
using api::LaneSRoute;
using api::rules::DirectionUsageRule;
using api::rules::DiscreteValueRule;
using api::rules::RangeValueRule;
using api::rules::RightOfWayRule;
using api::rules::RoadRulebook;
using api::rules::Rule;
using api::rules::SpeedLimitRule;
using api::test::CreateEmptyRelatedRules;
using api::test::CreateEmptyRelatedUniqueIds;
class ManualRulebookTest : public ::testing::Test {
 protected:
  const LaneSRange kZone{LaneId("a"), {10., 20.}};

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
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
#pragma GCC diagnostic pop

  const Rule::Id kDiscreteValueRuleId{"dvrt/dvr_id"};

  const DiscreteValueRule kDiscreteValueRule{
      kDiscreteValueRuleId,
      Rule::TypeId("dvrt"),
      LaneSRoute({kZone}),
      {DiscreteValueRule::DiscreteValue{Rule::State::kStrict, CreateEmptyRelatedRules(), CreateEmptyRelatedUniqueIds(),
                                        "value1"},
       DiscreteValueRule::DiscreteValue{Rule::State::kBestEffort, CreateEmptyRelatedRules(),
                                        CreateEmptyRelatedUniqueIds(), "value2"}}};

  const Rule::Id kRangeValueRuleId{"rvrt/rvr"};

  const RangeValueRule kRangeValueRule{
      kRangeValueRuleId,
      Rule::TypeId("rvrt"),
      LaneSRoute({kZone}),
      {RangeValueRule::Range{Rule::State::kStrict, CreateEmptyRelatedRules(), CreateEmptyRelatedUniqueIds(),
                             "description", 123., 456.}}};
};

TEST_F(ManualRulebookTest, DefaultConstructor) { ManualRulebook dut; }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
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
#pragma GCC diagnostic pop

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
      {DiscreteValueRule::DiscreteValue{Rule::State::kStrict, CreateEmptyRelatedRules(), CreateEmptyRelatedUniqueIds(),
                                        "value1"},
       DiscreteValueRule::DiscreteValue{Rule::State::kStrict, CreateEmptyRelatedRules(), CreateEmptyRelatedUniqueIds(),
                                        "value2"}}};
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

  const RangeValueRule kRangeValueRuleWithSameId{
      kDiscreteValueRuleId,
      Rule::TypeId("rvrt"),
      LaneSRoute({kZone}),
      {RangeValueRule::Range{Rule::State::kStrict, CreateEmptyRelatedRules(), CreateEmptyRelatedUniqueIds(),
                             "description", 123., 456.}}};

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
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  EXPECT_THROW(dut.GetRule(kRightOfWay.id()), std::out_of_range);
  EXPECT_THROW(dut.RemoveRule(kRightOfWay.id()), maliput::common::assertion_error);
  EXPECT_THROW(dut.GetRule(kSpeedLimit.id()), std::out_of_range);
  EXPECT_THROW(dut.RemoveRule(kSpeedLimit.id()), maliput::common::assertion_error);
  EXPECT_THROW(dut.GetRule(kDirectionUsage.id()), std::out_of_range);
  EXPECT_THROW(dut.RemoveRule(kDirectionUsage.id()), maliput::common::assertion_error);
#pragma GCC diagnostic pop
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
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut.GetRule(kRightOfWay.id()), kRightOfWay));
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut.GetRule(kSpeedLimit.id()), kSpeedLimit));
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut.GetRule(kDirectionUsage.id()), kDirectionUsage));
#pragma GCC diagnostic pop
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
  EXPECT_EQ(static_cast<int>(empty.right_of_way.size()), 0);
  EXPECT_EQ(static_cast<int>(empty.speed_limit.size()), 0);
  EXPECT_EQ(static_cast<int>(empty.direction_usage.size()), 0);
  EXPECT_EQ(static_cast<int>(empty.discrete_value_rules.size()), 0);
  EXPECT_EQ(static_cast<int>(empty.range_value_rules.size()), 0);

  const RoadRulebook::QueryResults nonempty = dut.FindRules({kZone}, kZeroTolerance);
  EXPECT_EQ(static_cast<int>(nonempty.right_of_way.size()), 1);
  EXPECT_EQ(static_cast<int>(nonempty.speed_limit.size()), 1);
  EXPECT_EQ(static_cast<int>(nonempty.direction_usage.size()), 1);
  EXPECT_EQ(static_cast<int>(nonempty.discrete_value_rules.size()), 1);
  EXPECT_EQ(static_cast<int>(nonempty.range_value_rules.size()), 1);

  const LaneSRange kReversedZone(kZone.lane_id(), {kZone.s_range().s1(), kZone.s_range().s0()});
  const RoadRulebook::QueryResults reversed = dut.FindRules({kReversedZone}, kZeroTolerance);
  EXPECT_EQ(static_cast<int>(reversed.right_of_way.size()), 1);
  EXPECT_EQ(static_cast<int>(reversed.speed_limit.size()), 1);
  EXPECT_EQ(static_cast<int>(reversed.direction_usage.size()), 1);
  EXPECT_EQ(static_cast<int>(reversed.discrete_value_rules.size()), 1);
  EXPECT_EQ(static_cast<int>(reversed.range_value_rules.size()), 1);

  const double kNonzeroTolerance = 0.1;

  ASSERT_LT(kZone.s_range().s0(), kZone.s_range().s1());
  // Construct a range that just barely overlaps the kNonzeroTolerance band
  // of kZone.s1.
  const LaneSRange kNearbyRange(kZone.lane_id(),
                                {kZone.s_range().s1() + (0.9 * kNonzeroTolerance), kZone.s_range().s1() + 50.});
  const RoadRulebook::QueryResults nearby = dut.FindRules({kNearbyRange}, kNonzeroTolerance);
  EXPECT_EQ(static_cast<int>(nearby.right_of_way.size()), 1);
  EXPECT_EQ(static_cast<int>(nearby.speed_limit.size()), 1);
  EXPECT_EQ(static_cast<int>(nearby.direction_usage.size()), 1);
  EXPECT_EQ(static_cast<int>(nearby.discrete_value_rules.size()), 1);
  EXPECT_EQ(static_cast<int>(nearby.range_value_rules.size()), 1);

  // Construct a range that sits just outside of the kNonzeroTolerance band
  // of kZone.s1.
  const LaneSRange kTooFarRange(kZone.lane_id(),
                                {kZone.s_range().s1() + (1.1 * kNonzeroTolerance), kZone.s_range().s1() + 50.});
  const RoadRulebook::QueryResults toofar = dut.FindRules({kTooFarRange}, kNonzeroTolerance);
  EXPECT_EQ(static_cast<int>(toofar.right_of_way.size()), 0);
  EXPECT_EQ(static_cast<int>(toofar.speed_limit.size()), 0);
  EXPECT_EQ(static_cast<int>(toofar.direction_usage.size()), 0);
  EXPECT_EQ(static_cast<int>(toofar.discrete_value_rules.size()), 0);
  EXPECT_EQ(static_cast<int>(toofar.range_value_rules.size()), 0);
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
  EXPECT_EQ(static_cast<int>(result.right_of_way.size()), 1);
  EXPECT_EQ(static_cast<int>(result.speed_limit.size()), 1);
  EXPECT_EQ(static_cast<int>(result.direction_usage.size()), 0);
  EXPECT_EQ(static_cast<int>(result.discrete_value_rules.size()), 0);
  EXPECT_EQ(static_cast<int>(result.range_value_rules.size()), 0);

  dut.AddRule(kDirectionUsage);
  result = dut.Rules();
  EXPECT_EQ(static_cast<int>(result.right_of_way.size()), 1);
  EXPECT_EQ(static_cast<int>(result.speed_limit.size()), 1);
  EXPECT_EQ(static_cast<int>(result.direction_usage.size()), 1);
  EXPECT_EQ(static_cast<int>(result.discrete_value_rules.size()), 0);
  EXPECT_EQ(static_cast<int>(result.range_value_rules.size()), 0);

  dut.AddRule(kDiscreteValueRule);
  result = dut.Rules();
  EXPECT_EQ(static_cast<int>(result.right_of_way.size()), 1);
  EXPECT_EQ(static_cast<int>(result.speed_limit.size()), 1);
  EXPECT_EQ(static_cast<int>(result.direction_usage.size()), 1);
  EXPECT_EQ(static_cast<int>(result.discrete_value_rules.size()), 1);
  EXPECT_EQ(static_cast<int>(result.range_value_rules.size()), 0);

  dut.AddRule(kRangeValueRule);
  result = dut.Rules();
  EXPECT_EQ(static_cast<int>(result.right_of_way.size()), 1);
  EXPECT_EQ(static_cast<int>(result.speed_limit.size()), 1);
  EXPECT_EQ(static_cast<int>(result.direction_usage.size()), 1);
  EXPECT_EQ(static_cast<int>(result.discrete_value_rules.size()), 1);
  EXPECT_EQ(static_cast<int>(result.range_value_rules.size()), 1);
}

}  // namespace
}  // namespace test
}  // namespace maliput
