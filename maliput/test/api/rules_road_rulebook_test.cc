/* clang-format off to disable clang-format-includes */
#include "maliput/api/rules/road_rulebook.h"
/* clang-format on */
// TODO(maddog@tri.global) Satisfy clang-format via rules tests directory reorg.

#include <gtest/gtest.h>

#include "maliput/api/regions.h"
#include "maliput/api/rules/direction_usage_rule.h"
#include "maliput/api/rules/right_of_way_rule.h"
#include "maliput/api/rules/speed_limit_rule.h"
#include "maliput/common/assertion_error.h"
#include "maliput/test_utilities/mock.h"

namespace maliput {
namespace api {
namespace rules {
namespace {

// This class does not provide any semblance of useful functionality.
// It merely exercises the RoadRulebook abstract interface.
class MockRulebook final : public RoadRulebook {
 public:
  const LaneSRange kZone{LaneId("some_lane"), {10., 20.}};
  const RightOfWayRule kRightOfWay{
      RightOfWayRule::Id("rowr_id"),
      LaneSRoute({kZone}),
      RightOfWayRule::ZoneType::kStopExcluded,
      {RightOfWayRule::State(RightOfWayRule::State::Id("green"), RightOfWayRule::State::Type::kGo, {} /* states */)},
      {} /* related_bulb_groups */};
  const SpeedLimitRule kSpeedLimit{SpeedLimitRule::Id("slr_id"), kZone, SpeedLimitRule::Severity::kStrict, 0., 44.};
  const DirectionUsageRule kDirectionUsage{
      DirectionUsageRule::Id("dur_id"),
      kZone,
      {DirectionUsageRule::State(DirectionUsageRule::State::Id("dur_state"), DirectionUsageRule::State::Type::kWithS,
                                 DirectionUsageRule::State::Severity::kPreferred)}};
  const RangeValueRule kRangeValueRule{
      Rule::Id("rvrt/rvr_id"), Rule::TypeId("rvrt"), LaneSRoute({kZone}), {api::test::CreateRange()}};
  const DiscreteValueRule kDiscreteValueRule{
      Rule::Id("dvrt/dvr_id"),
      Rule::TypeId("rvrt"),
      LaneSRoute({kZone}),
      {DiscreteValueRule::DiscreteValue{Rule::State::kStrict, api::test::CreateEmptyRelatedRules(),
                                        api::test::CreateEmptyRelatedUniqueIds(), "value1"},
       DiscreteValueRule::DiscreteValue{Rule::State::kBestEffort, api::test::CreateEmptyRelatedRules(),
                                        api::test::CreateEmptyRelatedUniqueIds(), "value2"}}};

 private:
  QueryResults DoFindRules(const std::vector<LaneSRange>& ranges, double) const override {
    QueryResults results;
    if ((!ranges.empty()) && (ranges[0].lane_id() == kZone.lane_id()) &&
        (ranges[0].s_range().s0() == kZone.s_range().s0()) && (ranges[0].s_range().s1() == kZone.s_range().s1())) {
      results.right_of_way.emplace(kRightOfWay.id(), kRightOfWay);
      results.speed_limit.emplace(kSpeedLimit.id(), kSpeedLimit);
      results.direction_usage.emplace(kDirectionUsage.id(), kDirectionUsage);
      results.discrete_value_rules.emplace(kDiscreteValueRule.id(), kDiscreteValueRule);
      results.range_value_rules.emplace(kRangeValueRule.id(), kRangeValueRule);
    }
    return results;
  }

  virtual QueryResults DoRules() const {
    return QueryResults{{{kRightOfWay.id(), kRightOfWay}},
                        {{kSpeedLimit.id(), kSpeedLimit}},
                        {{kDirectionUsage.id(), kDirectionUsage}},
                        {{kDiscreteValueRule.id(), kDiscreteValueRule}},
                        {{kRangeValueRule.id(), kRangeValueRule}}};
  }

  virtual RightOfWayRule DoGetRule(const RightOfWayRule::Id& id) const {
    if (id != kRightOfWay.id()) {
      throw std::out_of_range("");
    }
    return kRightOfWay;
  }

  SpeedLimitRule DoGetRule(const SpeedLimitRule::Id& id) const override {
    if (id != kSpeedLimit.id()) {
      throw std::out_of_range("");
    }
    return kSpeedLimit;
  }

  DirectionUsageRule DoGetRule(const DirectionUsageRule::Id& id) const override {
    if (id != kDirectionUsage.id()) {
      throw std::out_of_range("");
    }
    return kDirectionUsage;
  }

  DiscreteValueRule DoGetDiscreteValueRule(const Rule::Id& id) const override {
    if (id != kDiscreteValueRule.id()) {
      throw std::out_of_range("");
    }
    return kDiscreteValueRule;
  }

  RangeValueRule DoGetRangeValueRule(const Rule::Id& id) const override {
    if (id != kRangeValueRule.id()) {
      throw std::out_of_range("");
    }
    return kRangeValueRule;
  }
};

GTEST_TEST(RoadRulebookTest, ExerciseInterface) {
  const MockRulebook dut;

  const double kZeroTolerance = 0.;

  RoadRulebook::QueryResults nonempty = dut.FindRules({dut.kZone}, kZeroTolerance);
  EXPECT_EQ(nonempty.right_of_way.size(), 1);
  EXPECT_EQ(nonempty.speed_limit.size(), 1);
  EXPECT_EQ(nonempty.direction_usage.size(), 1);
  EXPECT_EQ(nonempty.discrete_value_rules.size(), 1);
  EXPECT_EQ(nonempty.range_value_rules.size(), 1);

  RoadRulebook::QueryResults empty = dut.FindRules({}, kZeroTolerance);
  EXPECT_EQ(empty.right_of_way.size(), 0);
  EXPECT_EQ(empty.speed_limit.size(), 0);
  EXPECT_EQ(empty.direction_usage.size(), 0);
  EXPECT_EQ(empty.discrete_value_rules.size(), 0);
  EXPECT_EQ(empty.range_value_rules.size(), 0);

  const double kNegativeTolerance = -1.;
  EXPECT_THROW(dut.FindRules({}, kNegativeTolerance), maliput::common::assertion_error);

  nonempty = dut.Rules();
  EXPECT_EQ(nonempty.right_of_way.size(), 1);
  EXPECT_EQ(nonempty.speed_limit.size(), 1);
  EXPECT_EQ(nonempty.direction_usage.size(), 1);
  EXPECT_EQ(nonempty.discrete_value_rules.size(), 1);
  EXPECT_EQ(nonempty.range_value_rules.size(), 1);

  EXPECT_EQ(dut.GetRule(dut.kRightOfWay.id()).id(), dut.kRightOfWay.id());
  EXPECT_THROW(dut.GetRule(RightOfWayRule::Id("xxx")), std::out_of_range);

  EXPECT_EQ(dut.GetRule(dut.kSpeedLimit.id()).id(), dut.kSpeedLimit.id());
  EXPECT_THROW(dut.GetRule(SpeedLimitRule::Id("xxx")), std::out_of_range);

  EXPECT_EQ(dut.GetRule(dut.kDirectionUsage.id()).id(), dut.kDirectionUsage.id());
  EXPECT_THROW(dut.GetRule(DirectionUsageRule::Id("xxx")), std::out_of_range);

  EXPECT_EQ(dut.GetDiscreteValueRule(dut.kDiscreteValueRule.id()).id(), dut.kDiscreteValueRule.id());
  EXPECT_THROW(dut.GetDiscreteValueRule(Rule::Id("xxx")), std::out_of_range);

  EXPECT_EQ(dut.GetRangeValueRule(dut.kRangeValueRule.id()).id(), dut.kRangeValueRule.id());
  EXPECT_THROW(dut.GetRangeValueRule(Rule::Id("xxx")), std::out_of_range);
}

}  // namespace
}  // namespace rules
}  // namespace api
}  // namespace maliput
