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
#include "maliput/base/rule_filter.h"

#include <map>

#include <gtest/gtest.h>

#include "maliput/api/lane.h"

using maliput::api::LaneId;
using maliput::api::LaneSRange;
using maliput::api::LaneSRoute;
using maliput::api::rules::DiscreteValueRule;
using maliput::api::rules::RangeValueRule;
using maliput::api::rules::Rule;

namespace maliput {
namespace test {
namespace {

template <typename T>
struct AlwaysTrue {
  bool operator()(const T&) const { return true; }
};

template <typename T>
struct AlwaysFalse {
  bool operator()(const T&) const { return false; }
};

GTEST_TEST(FilterRuleTest, BasicTest) {
  const api::rules::RoadRulebook::QueryResults query_result{
      {} /* right_of_way */,
      {} /* speed_limit */,
      {} /* direction_usage */,
      std::map<DiscreteValueRule::Id, DiscreteValueRule>{
          {Rule::Id("dvrt a/1"),
           DiscreteValueRule(Rule::Id("dvrt a/1"), Rule::TypeId("dvrt a"),
                             LaneSRoute({LaneSRange{LaneId("a"), {10., 20.}}}),
                             {DiscreteValueRule::DiscreteValue{Rule::State::kStrict, Rule::RelatedRules{},
                                                               Rule::RelatedUniqueIds{}, "ValueA"}})},
          {Rule::Id("dvrt a/1"),
           DiscreteValueRule(Rule::Id("dvrt b/2"), Rule::TypeId("dvrt b"),
                             LaneSRoute({LaneSRange{LaneId("a"), {10., 20.}}}),
                             {DiscreteValueRule::DiscreteValue{Rule::State::kBestEffort, Rule::RelatedRules{},
                                                               Rule::RelatedUniqueIds{}, "ValueB"}})},
      },
      std::map<RangeValueRule::Id, RangeValueRule>{
          {Rule::Id("rvrt a/1"),
           RangeValueRule(Rule::Id("rvrt a/1"), Rule::TypeId("rvrt a"),
                          LaneSRoute({LaneSRange{LaneId("a"), {10., 20.}}}),
                          {RangeValueRule::Range{Rule::State::kStrict, Rule::RelatedRules{}, Rule::RelatedUniqueIds{},
                                                 "Range description A", 123., 456.}})},
          {Rule::Id("rvrt b/2"),
           RangeValueRule(Rule::Id("rvrt b/2"), Rule::TypeId("rvrt b"),
                          LaneSRoute({LaneSRange{LaneId("a"), {10., 20.}}}),
                          {RangeValueRule::Range{Rule::State::kStrict, Rule::RelatedRules{}, Rule::RelatedUniqueIds{},
                                                 "Range description B", 789., 1234.}})},
      }};

  const api::rules::RoadRulebook::QueryResults empty_result =
      FilterRules(query_result, {AlwaysFalse<DiscreteValueRule>()}, {AlwaysFalse<RangeValueRule>()});
  EXPECT_TRUE(empty_result.discrete_value_rules.empty());
  EXPECT_TRUE(empty_result.range_value_rules.empty());

  const api::rules::RoadRulebook::QueryResults full_result =
      FilterRules(query_result, {AlwaysTrue<DiscreteValueRule>()}, {AlwaysTrue<RangeValueRule>()});
  EXPECT_EQ(full_result.discrete_value_rules.size(), query_result.discrete_value_rules.size());
  EXPECT_EQ(full_result.range_value_rules.size(), query_result.range_value_rules.size());
}

}  // namespace
}  // namespace test
}  // namespace maliput
