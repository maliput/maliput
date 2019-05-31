/* clang-format off to disable clang-format-includes */
#include "maliput/api/rules/road_rulebook.h"
/* clang-format on */
// TODO(maddog@tri.global) Satisfy clang-format via rules tests directory reorg.

#include <memory>
#include <string>
#include <utility>

#include <gtest/gtest.h>

#include "maliput/api/rules/direction_usage_rule.h"
#include "maliput/api/rules/regions.h"
#include "maliput/api/rules/right_of_way_rule.h"
#include "maliput/api/rules/speed_limit_rule.h"
#include "maliput/api/rules/rule.h"
#include "drake/common/drake_throw.h"

namespace maliput {
namespace api {
namespace rules {
namespace {

// This class does not provide any semblance of useful functionality.
// It merely exercises the RoadRulebook abstract interface.
class MockRuleStateType : public RuleStateType {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockRuleStateType)

  const int kMockValue{123};
  const static std::string kMockString;

  static std::unique_ptr<MockRuleStateType> Mock();

 private:
  MockRuleStateType() : RuleStateType(kMockValue, kMockString) {}
};

const std::string MockRuleStateType::kMockString{"mock_rule_state_type"};

std::unique_ptr<MockRuleStateType> MockRuleStateType::Mock() {
  return std::unique_ptr<MockRuleStateType>(new MockRuleStateType());
}

// This class does not provide any semblance of useful functionality.
// It merely exercises the RoadRulebook abstract interface.
class MockRulebook : public RoadRulebook {
 public:
  MockRulebook() : RoadRulebook() {
    std::vector<std::unique_ptr<RuleState>> states;
    states.push_back(std::make_unique<RuleState>(
        RuleState::Id("rs_id"), RuleState::Severity::kPreferred,
        MockRuleStateType::Mock()));

    mock_rule = std::make_unique<RuleBase>(
        RuleBase::Id("r_id"), kZone, RuleBase::RuleTypeId("r_type"),
        std::move(states));

    std::vector<std::unique_ptr<RuleBase>> rules;
    states.clear();
    states.push_back(std::make_unique<RuleState>(
        RuleState::Id("rs_id"), RuleState::Severity::kPreferred,
        MockRuleStateType::Mock()));
    rules.push_back(std::make_unique<RuleBase>(
        RuleBase::Id("r_id"), kZone, RuleBase::RuleTypeId("r_type"),
        std::move(states)));
    mock_rule_group =
        std::make_unique<RuleGroup>(RuleGroup::Id("rg_id"), std::move(rules));
  }

  const LaneSRange kZone{LaneId("some_lane"), {10., 20.}};
  const RightOfWayRule kRightOfWay{
    RightOfWayRule::Id("rowr_id"),
    LaneSRoute({kZone}), RightOfWayRule::ZoneType::kStopExcluded,
    {RightOfWayRule::State(
        RightOfWayRule::State::Id("green"),
        RightOfWayRule::State::Type::kGo,
        {})}};
  const SpeedLimitRule kSpeedLimit{SpeedLimitRule::Id("slr_id"),
                                   kZone,
                                   SpeedLimitRule::Severity::kStrict,
                                   0., 44.};
  const DirectionUsageRule kDirectionUsage{
    DirectionUsageRule::Id("dur_id"), kZone,
    {DirectionUsageRule::State(
      DirectionUsageRule::State::Id("dur_state"),
      DirectionUsageRule::State::Type::kWithS,
      DirectionUsageRule::State::Severity::kPreferred)}};

  std::unique_ptr<RuleBase> mock_rule;
  std::unique_ptr<RuleGroup> mock_rule_group;

 private:
  virtual QueryResults DoFindRules(
      const std::vector<LaneSRange>& ranges, double) const {
    QueryResults results;
    if ((!ranges.empty()) &&
        (ranges[0].lane_id() == kZone.lane_id()) &&
        (ranges[0].s_range().s0() == kZone.s_range().s0()) &&
        (ranges[0].s_range().s1() == kZone.s_range().s1())) {
      results.right_of_way.push_back(kRightOfWay);
      results.speed_limit.push_back(kSpeedLimit);
      results.direction_usage.push_back(kDirectionUsage);
    }
    return results;
  }

  virtual RightOfWayRule DoGetRule(const RightOfWayRule::Id& id) const {
    if (id != kRightOfWay.id()) {
      throw std::out_of_range("");
    }
    return kRightOfWay;
  }

  virtual SpeedLimitRule DoGetRule(const SpeedLimitRule::Id& id) const {
    if (id != kSpeedLimit.id()) {
      throw std::out_of_range("");
    }
    return kSpeedLimit;
  }

  virtual DirectionUsageRule DoGetRule(const DirectionUsageRule::Id& id) const {
    if (id != kDirectionUsage.id()) {
      throw std::out_of_range("");
    }
    return kDirectionUsage;
  }

  virtual RuleBase* DoGetRule(const RuleBase::Id& id) const {
    if (id != mock_rule->id()) {
      throw std::out_of_range("");
    }
    return mock_rule.get();
  }

  virtual RuleGroup* DoGetRuleGroup(const RuleGroup::Id& id) const {
    if (id != mock_rule_group->id()) {
      throw std::out_of_range("");
    }
    return mock_rule_group.get();
  }
};


GTEST_TEST(RoadRulebookTest, ExerciseInterface) {
  const MockRulebook dut;

  const double kZeroTolerance = 0.;

  RoadRulebook::QueryResults nonempty = dut.FindRules({dut.kZone},
                                                      kZeroTolerance);
  EXPECT_EQ(nonempty.right_of_way.size(), 1);
  EXPECT_EQ(nonempty.speed_limit.size(), 1);
  EXPECT_EQ(nonempty.direction_usage.size(), 1);
  RoadRulebook::QueryResults empty = dut.FindRules({}, kZeroTolerance);
  EXPECT_EQ(empty.right_of_way.size(), 0);
  EXPECT_EQ(empty.speed_limit.size(), 0);
  EXPECT_EQ(empty.direction_usage.size(), 0);

  const double kNegativeTolerance = -1.;
  EXPECT_THROW(dut.FindRules({}, kNegativeTolerance),
               std::runtime_error);

  EXPECT_EQ(dut.GetRule(dut.kRightOfWay.id()).id(), dut.kRightOfWay.id());
  EXPECT_THROW(dut.GetRule(RightOfWayRule::Id("xxx")), std::out_of_range);

  EXPECT_EQ(dut.GetRule(dut.kSpeedLimit.id()).id(), dut.kSpeedLimit.id());
  EXPECT_THROW(dut.GetRule(SpeedLimitRule::Id("xxx")), std::out_of_range);

  EXPECT_EQ(dut.GetRule(dut.kDirectionUsage.id()).id(),
                        dut.kDirectionUsage.id());
  EXPECT_THROW(dut.GetRule(DirectionUsageRule::Id("xxx")),
                           std::out_of_range);

  EXPECT_EQ(dut.GetRule(dut.mock_rule->id())->id(), dut.mock_rule->id());
  EXPECT_THROW(dut.GetRule(RuleBase::Id("xxx")), std::out_of_range);
  EXPECT_EQ(dut.GetRuleGroup(dut.mock_rule_group->id())->id(),
            dut.mock_rule_group->id());
  EXPECT_THROW(dut.GetRuleGroup(RuleGroup::Id("xxx")), std::out_of_range);
}


}  // namespace
}  // namespace rules
}  // namespace api
}  // namespace maliput
