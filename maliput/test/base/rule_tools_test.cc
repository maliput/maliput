#include "maliput/base/rule_tools.h"

#include <gtest/gtest.h>

#include "maliput/api/lane.h"
#include "maliput/api/rules/road_rulebook.h"
#include "maliput/base/rule_filter.h"

using maliput::api::LaneId;
using maliput::api::LaneSRange;
using maliput::api::LaneSRoute;
using maliput::api::rules::DiscreteValueRule;
using maliput::api::rules::MakeDiscreteValue;
using maliput::api::rules::MakeRange;
using maliput::api::rules::RangeValueRule;
using maliput::api::rules::Rule;

namespace maliput {
namespace test {
namespace {

GTEST_TEST(FilterRuleByRuleTypeIdTest, BasicTest) {
  const api::rules::RoadRulebook::QueryResults query_result{
      {} /* right_of_way */,
      {} /* speed_limit */,
      {} /* direction_usage */,
      std::map<DiscreteValueRule::Id, DiscreteValueRule>{
          {Rule::Id("dvrt a/1"),
           DiscreteValueRule(
               Rule::Id("dvrt a/1"), Rule::TypeId("dvrt a"), LaneSRoute({LaneSRange{LaneId("a"), {10., 20.}}}),
               {MakeDiscreteValue(Rule::State::kStrict, Rule::RelatedRules{}, Rule::RelatedUniqueIds{}, "ValueA")})},
          {Rule::Id("dvrt a/1"), DiscreteValueRule(Rule::Id("dvrt b/2"), Rule::TypeId("dvrt b"),
                                                   LaneSRoute({LaneSRange{LaneId("a"), {10., 20.}}}),
                                                   {MakeDiscreteValue(Rule::State::kBestEffort, Rule::RelatedRules{},
                                                                      Rule::RelatedUniqueIds{}, "ValueB")})},
      },
      std::map<RangeValueRule::Id, RangeValueRule>{
          {Rule::Id("rvrt a/1"),
           RangeValueRule(Rule::Id("rvrt a/1"), Rule::TypeId("rvrt a"),
                          LaneSRoute({LaneSRange{LaneId("a"), {10., 20.}}}),
                          {MakeRange(Rule::State::kStrict, Rule::RelatedRules{}, Rule::RelatedUniqueIds{},
                                     "Range description A", 123., 456.)})},
          {Rule::Id("rvrt b/2"),
           RangeValueRule(Rule::Id("rvrt b/2"), Rule::TypeId("rvrt b"),
                          LaneSRoute({LaneSRange{LaneId("a"), {10., 20.}}}),
                          {MakeRange(Rule::State::kStrict, Rule::RelatedRules{}, Rule::RelatedUniqueIds{},
                                     "Range description B", 789., 1234.)})},
      }};

  const api::rules::RoadRulebook::QueryResults dut =
      FilterRules(query_result, {RuleTypeFilter(Rule::TypeId("dvrt a"))}, {});
  EXPECT_EQ(dut.range_value_rules.size(), query_result.range_value_rules.size());
  EXPECT_EQ(dut.discrete_value_rules.size(), 1);
  EXPECT_NE(dut.discrete_value_rules.find(Rule::Id(Rule::Id("dvrt a/1"))), dut.discrete_value_rules.end());
}

}  // namespace
}  // namespace test
}  // namespace maliput
