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
#include <gtest/gtest.h>

#include "maliput/api/rules/discrete_value_rule.h"
#include "maliput/api/rules/range_value_rule.h"
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
      RangeValueRule::Range{Rule::State::kStrict, api::test::CreateEmptyRelatedRules(),
                            api::test::CreateEmptyRelatedUniqueIds(), "range_description_1", 123. /* min */,
                            456. /* max */},
      RangeValueRule::Range{Rule::State::kBestEffort, api::test::CreateEmptyRelatedRules(),
                            api::test::CreateEmptyRelatedUniqueIds(), "range_description_2", 789. /* min */,
                            1234. /* max */},
      RangeValueRule::Range{Rule::State::kStrict, api::test::CreateNonEmptyRelatedRules(),
                            api::test::CreateNonEmptyRelatedUniqueIds(), "range_description_3", 123. /* min */,
                            456. /* max */},
      RangeValueRule::Range{Rule::State::kBestEffort, api::test::CreateNonEmptyRelatedRules(),
                            api::test::CreateNonEmptyRelatedUniqueIds(), "range_description_4", 789. /* min */,
                            1234. /* max */},
  };

  EXPECT_NO_THROW(RangeValueRule(kId, kTypeId, kZone, kRanges));

  // Missing Rule::Ids in RelatedRules in RangeValueRule::Range.
  const Rule::RelatedRules kMissingRelatedRules{{"RuleGroup", {}}};
  const RangeValueRule::Range kRangeWithMissingRelatedRules =
      RangeValueRule::Range{Rule::State::kStrict,  kMissingRelatedRules, api::test::CreateEmptyRelatedUniqueIds(),
                            "range_description_1", 123. /* min */,       456. /* max */};
  EXPECT_NO_THROW(RangeValueRule(kId, kTypeId, kZone, {kRangeWithMissingRelatedRules}));

  // Duplicated Rule::Ids in RelatedRules in RangeValueRule::Range.
  const Rule::RelatedRules kDuplicatedRelatedRules{
      {"RuleGroup", {Rule::Id("RuleTypeIdB/RuleIdB"), Rule::Id("RuleTypeIdB/RuleIdB")}}};
  const RangeValueRule::Range kRangeWithDuplicatedRelatedRulesIds =
      RangeValueRule::Range{Rule::State::kStrict,  kDuplicatedRelatedRules, api::test::CreateEmptyRelatedUniqueIds(),
                            "range_description_1", 123. /* min */,          456. /* max */};
  EXPECT_THROW(RangeValueRule(kId, kTypeId, kZone, {kRangeWithDuplicatedRelatedRulesIds}),
               maliput::common::assertion_error);

  // Duplicated UniqueIds in RelatedUniqueIds in RangeValueRule::Range.
  const Rule::RelatedUniqueIds kDuplicatedRelatedUniqueIds{
      {"UniqueBulbGroupId",
       {rules::UniqueBulbGroupId(TrafficLight::Id("TrafficLightIdA"), BulbGroup::Id("BulbGroupIdA")),
        rules::UniqueBulbGroupId(TrafficLight::Id("TrafficLightIdA"), BulbGroup::Id("BulbGroupIdA"))}}};
  const RangeValueRule::Range kRangeWithDuplicatedRelatedUniqueIds =
      RangeValueRule::Range{Rule::State::kStrict,
                            api::test::CreateEmptyRelatedRules(),
                            kDuplicatedRelatedUniqueIds,
                            "range_description_1",
                            123. /* min */,
                            456. /* max */};
  EXPECT_THROW(RangeValueRule(kId, kTypeId, kZone, {kRangeWithDuplicatedRelatedUniqueIds}),
               maliput::common::assertion_error);

  // Empty std::string for semantic group key in RelatedRules in RangeValueRule::Range.
  const Rule::RelatedRules kEmptyKeyRelatedRules{{"", {Rule::Id("RuleTypeIdB/RuleIdB")}}};
  const RangeValueRule::Range kRangeWithEmptyKeyRelatedRules =
      RangeValueRule::Range{Rule::State::kStrict,  kEmptyKeyRelatedRules, api::test::CreateEmptyRelatedUniqueIds(),
                            "range_description_1", 123. /* min */,        456. /* max */};
  EXPECT_THROW(RangeValueRule(kId, kTypeId, kZone, {kRangeWithEmptyKeyRelatedRules}), maliput::common::assertion_error);

  // Empty std::string for semantic group key in RelatedUniqueIds in RangeValueRule::Range.
  const Rule::RelatedUniqueIds kEmptyKeyRelatedUniqueIds{
      {"", {rules::UniqueBulbGroupId(TrafficLight::Id("TrafficLightIdB"), BulbGroup::Id("BulbGroupIdB"))}}};
  const RangeValueRule::Range kRangeWithEmptyKeyRelatedUniqueIds =
      RangeValueRule::Range{Rule::State::kStrict,
                            api::test::CreateEmptyRelatedRules(),
                            kEmptyKeyRelatedUniqueIds,
                            "range_description_1",
                            123. /* min */,
                            456. /* max */};
  EXPECT_THROW(RangeValueRule(kId, kTypeId, kZone, {kRangeWithEmptyKeyRelatedUniqueIds}),
               maliput::common::assertion_error);

  // Negative severity.
  const int kSeverityInvalid{-1};
  EXPECT_THROW(RangeValueRule(kId, kTypeId, kZone,
                              {RangeValueRule::Range{kSeverityInvalid, api::test::CreateNonEmptyRelatedRules(),
                                                     api::test::CreateEmptyRelatedUniqueIds(), "range_description_1",
                                                     123. /* min */, 456. /* max */}}),
               maliput::common::assertion_error);
  // Empty ranges.
  EXPECT_THROW(RangeValueRule(kId, kTypeId, kZone, {} /* ranges */), maliput::common::assertion_error);
  // Duplicated ranges.
  const std::vector<RangeValueRule::Range> kDuplicatedRanges{
      RangeValueRule::Range{Rule::State::kStrict, api::test::CreateEmptyRelatedRules(),
                            api::test::CreateEmptyRelatedUniqueIds(), "range_description_1", 123. /* min */,
                            456. /* max */},
      RangeValueRule::Range{Rule::State::kStrict, api::test::CreateEmptyRelatedRules(),
                            api::test::CreateEmptyRelatedUniqueIds(), "range_description_1", 123. /* min */,
                            456. /* max */},
  };
  EXPECT_THROW(RangeValueRule(kId, kTypeId, kZone, kDuplicatedRanges), maliput::common::assertion_error);

  // RangeValueRule::Range::min is greater than RangeValueRule::Range::max.
  const std::vector<RangeValueRule::Range> kShiftedRanges{RangeValueRule::Range{
      Rule::State::kStrict, api::test::CreateEmptyRelatedRules(), api::test::CreateEmptyRelatedUniqueIds(),
      "range_description_3", 456. /* min */, 123. /* max */}};
  EXPECT_THROW(RangeValueRule(kId, kTypeId, kZone, {kShiftedRanges}), maliput::common::assertion_error);
}

// Evaluates RangeValueRule accessors.
TEST_F(RuleTest, RangeValueRuleAccessors) {
  const std::vector<RangeValueRule::Range> kRanges{
      RangeValueRule::Range{Rule::State::kStrict, api::test::CreateEmptyRelatedRules(),
                            api::test::CreateEmptyRelatedUniqueIds(), "range_description_1", 123. /* min */,
                            456. /* max */},
      RangeValueRule::Range{Rule::State::kBestEffort, api::test::CreateNonEmptyRelatedRules(),
                            api::test::CreateEmptyRelatedUniqueIds(), "range_description_2", 789. /* min */,
                            1234. /* max */},
      RangeValueRule::Range{Rule::State::kStrict, api::test::CreateEmptyRelatedRules(),
                            api::test::CreateNonEmptyRelatedUniqueIds(), "range_description_3", 567. /* min */,
                            891. /* max */},
      RangeValueRule::Range{Rule::State::kBestEffort, api::test::CreateNonEmptyRelatedRules(),
                            api::test::CreateNonEmptyRelatedUniqueIds(), "range_description_4", 2345. /* min */,
                            6789. /* max */},
  };

  const RangeValueRule dut(kId, kTypeId, kZone, kRanges);

  EXPECT_EQ(dut.id(), kId);
  EXPECT_EQ(dut.type_id(), kTypeId);
  EXPECT_TRUE(MALIPUT_REGIONS_IS_EQUAL(dut.zone(), kZone));
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut.states(), kRanges));
}

// Evaluates the equal and not equal operator overloads for
// RangeValueRule::Range.
GTEST_TEST(RangeTest, EqualOperator) {
  const RangeValueRule::Range range_1{Rule::State::kStrict,
                                      api::test::CreateEmptyRelatedRules(),
                                      api::test::CreateEmptyRelatedUniqueIds(),
                                      "range_description_1",
                                      123. /* min */,
                                      456. /* max */};
  const RangeValueRule::Range range_2{Rule::State::kStrict,
                                      api::test::CreateEmptyRelatedRules(),
                                      api::test::CreateEmptyRelatedUniqueIds(),
                                      "range_description_1",
                                      456. /* min */,
                                      456. /* max */};
  const RangeValueRule::Range range_3{Rule::State::kStrict,
                                      api::test::CreateEmptyRelatedRules(),
                                      api::test::CreateEmptyRelatedUniqueIds(),
                                      "range_description_1",
                                      123. /* min */,
                                      789. /* max */};
  const RangeValueRule::Range range_4{Rule::State::kStrict,
                                      api::test::CreateEmptyRelatedRules(),
                                      api::test::CreateEmptyRelatedUniqueIds(),
                                      "range_description_4",
                                      123. /* min */,
                                      456. /* max */};
  const RangeValueRule::Range range_5{Rule::State::kBestEffort,
                                      api::test::CreateEmptyRelatedRules(),
                                      api::test::CreateEmptyRelatedUniqueIds(),
                                      "range_description_1",
                                      123. /* min */,
                                      456. /* max */};
  const RangeValueRule::Range range_6{Rule::State::kStrict,
                                      api::test::CreateNonEmptyRelatedRules(),
                                      api::test::CreateNonEmptyRelatedUniqueIds(),
                                      "range_description_6",
                                      123. /* min */,
                                      456. /* max */};

  // `related_rules` and `related_unique_ids` are empty.
  EXPECT_TRUE(range_1 == range_1);
  EXPECT_FALSE(range_1 != range_1);
  // `related_rules` and `related_unique_ids` are not empty.
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

// Evaluates the less operator of RangeValueRule::Range.
GTEST_TEST(RangeTest, LessOperator) {
  const RangeValueRule::Range range_1{Rule::State::kStrict,
                                      api::test::CreateEmptyRelatedRules(),
                                      api::test::CreateEmptyRelatedUniqueIds(),
                                      "range_description",
                                      123. /* min */,
                                      456. /* max */};
  const RangeValueRule::Range range_2{Rule::State::kBestEffort,
                                      api::test::CreateEmptyRelatedRules(),
                                      api::test::CreateEmptyRelatedUniqueIds(),
                                      "range_description_1",
                                      123. /* min */,
                                      456. /* max */};
  const RangeValueRule::Range range_3{Rule::State::kStrict,
                                      api::test::CreateEmptyRelatedRules(),
                                      api::test::CreateEmptyRelatedUniqueIds(),
                                      "range_description_long",
                                      123. /* min */,
                                      456. /* max */};
  const RangeValueRule::Range range_4{Rule::State::kStrict,
                                      api::test::CreateEmptyRelatedRules(),
                                      api::test::CreateEmptyRelatedUniqueIds(),
                                      "range_description",
                                      456. /* min */,
                                      456. /* max */};
  const RangeValueRule::Range range_5{Rule::State::kStrict,
                                      api::test::CreateEmptyRelatedRules(),
                                      api::test::CreateEmptyRelatedUniqueIds(),
                                      "range_description",
                                      123. /* min */,
                                      789. /* max */};
  const RangeValueRule::Range range_6{Rule::State::kStrict,
                                      api::test::CreateNonEmptyRelatedRules(),
                                      api::test::CreateNonEmptyRelatedUniqueIds(),
                                      "range_description",
                                      123. /* min */,
                                      456. /* max */};

  // Ranges are equal.
  EXPECT_FALSE(range_1 < range_1);
  // Evaluates by `severity`.
  EXPECT_TRUE(range_1 < range_2);
  // Evaluates by `description`.
  EXPECT_TRUE(range_1 < range_3);
  // Evaluates by `min`.
  EXPECT_TRUE(range_1 < range_4);
  // Evaluates by `max`.
  EXPECT_TRUE(range_1 < range_5);
  // Ranges are equal for this operator, because `related_rules` and `related_unique_ids` are not taken
  // into account.
  EXPECT_FALSE(range_1 < range_6);
  EXPECT_FALSE(range_6 < range_1);
}

// Evaluates DiscreteValueRule constructor.
TEST_F(RuleTest, DiscreteValueRuleConstructor) {
  const std::vector<DiscreteValueRule::DiscreteValue> kDiscreteValues{
      DiscreteValueRule::DiscreteValue{Rule::State::kStrict, api::test::CreateEmptyRelatedRules(),
                                       api::test::CreateEmptyRelatedUniqueIds(), "rule_state_value_1"},
      DiscreteValueRule::DiscreteValue{Rule::State::kBestEffort, api::test::CreateNonEmptyRelatedRules(),
                                       api::test::CreateEmptyRelatedUniqueIds(), "rule_state_value_2"},
      DiscreteValueRule::DiscreteValue{Rule::State::kStrict, api::test::CreateEmptyRelatedRules(),
                                       api::test::CreateNonEmptyRelatedUniqueIds(), "rule_state_value_3"},
      DiscreteValueRule::DiscreteValue{Rule::State::kBestEffort, api::test::CreateNonEmptyRelatedRules(),
                                       api::test::CreateNonEmptyRelatedUniqueIds(), "rule_state_value_4"}};
  EXPECT_NO_THROW(DiscreteValueRule(kId, kTypeId, kZone, kDiscreteValues));

  // Missing Rule::Ids in RelatedRules in DiscreteValueRule::DiscreteValue.
  const Rule::RelatedRules kMissingRelatedRules{{"RuleGroup", {}}};
  const DiscreteValueRule::DiscreteValue kDiscreteValueWithMissingRelatedRules{
      Rule::State::kStrict, kMissingRelatedRules, api::test::CreateEmptyRelatedUniqueIds(), "rule_state_value"};
  EXPECT_NO_THROW(DiscreteValueRule(kId, kTypeId, kZone, {kDiscreteValueWithMissingRelatedRules}));

  // Negative severity.
  const int kSeverityInvalid{-1};
  EXPECT_THROW(DiscreteValueRule(
                   kId, kTypeId, kZone,
                   {DiscreteValueRule::DiscreteValue{kSeverityInvalid, api::test::CreateEmptyRelatedRules(),
                                                     api::test::CreateEmptyRelatedUniqueIds(), "rule_state_value"}}),
               maliput::common::assertion_error);

  // Duplicated Rule::Ids in RelatedRules in DiscreteValueRule::DiscreteValue.
  const Rule::RelatedRules kDuplicatedRelatedRules{
      {"RuleGroup", {Rule::Id("RuleTypeIdB/RuleIdB"), Rule::Id("RuleTypeIdB/RuleIdB")}}};
  const DiscreteValueRule::DiscreteValue kDiscreteValueWithDuplicatedRules = DiscreteValueRule::DiscreteValue{
      Rule::State::kStrict, kDuplicatedRelatedRules, api::test::CreateEmptyRelatedUniqueIds(), "rule_state_value"};
  EXPECT_THROW(DiscreteValueRule(kId, kTypeId, kZone, {kDiscreteValueWithDuplicatedRules}),
               maliput::common::assertion_error);

  // Duplicated UniqueIds in RelatedUniqueIds in DiscreteValueRule::DiscreteValue.
  const Rule::RelatedUniqueIds kDuplicatedRelatedUniqueIds{
      {"UniqueBulbGroupId",
       {rules::UniqueBulbGroupId(TrafficLight::Id("TrafficLightIdA"), BulbGroup::Id("BulbGroupIdA")),
        rules::UniqueBulbGroupId(TrafficLight::Id("TrafficLightIdA"), BulbGroup::Id("BulbGroupIdA"))}}};
  const DiscreteValueRule::DiscreteValue kDiscreteValueWithDuplicatedRelatedUniqueIds =
      DiscreteValueRule::DiscreteValue{Rule::State::kStrict, api::test::CreateEmptyRelatedRules(),
                                       kDuplicatedRelatedUniqueIds, "rule_state_value"};
  EXPECT_THROW(DiscreteValueRule(kId, kTypeId, kZone, {kDiscreteValueWithDuplicatedRelatedUniqueIds}),
               maliput::common::assertion_error);

  // Duplicated discrete values.
  const std::vector<DiscreteValueRule::DiscreteValue> kDuplicatedDiscreteValues{
      DiscreteValueRule::DiscreteValue{Rule::State::kStrict, api::test::CreateNonEmptyRelatedRules(),
                                       api::test::CreateEmptyRelatedUniqueIds(), "rule_state_value_1"},
      DiscreteValueRule::DiscreteValue{Rule::State::kStrict, api::test::CreateNonEmptyRelatedRules(),
                                       api::test::CreateEmptyRelatedUniqueIds(), "rule_state_value_1"}};
  EXPECT_THROW(DiscreteValueRule(kId, kTypeId, kZone, kDuplicatedDiscreteValues), maliput::common::assertion_error);

  // Empty std::string for semantic group key in RelatedRules in DiscreteValueRule::DiscreteValue.
  const Rule::RelatedRules kEmptyKeyRelatedRules{{"", {Rule::Id("RuleTypeIdB/RuleIdB")}}};
  const DiscreteValueRule::DiscreteValue kDiscreteValueWithEmptyKeyRelatedRules = DiscreteValueRule::DiscreteValue{
      Rule::State::kStrict, kEmptyKeyRelatedRules, api::test::CreateEmptyRelatedUniqueIds(), "rule_state_value"};
  EXPECT_THROW(DiscreteValueRule(kId, kTypeId, kZone, {kDiscreteValueWithEmptyKeyRelatedRules}),
               maliput::common::assertion_error);

  // Empty std::string for semantic group key in RelatedUniqueIds in DiscreteValueRule::DiscreteValue.
  const Rule::RelatedUniqueIds kEmptyKeyRelatedUniqueIds{
      {"", {rules::UniqueBulbGroupId(TrafficLight::Id("TrafficLightIdB"), BulbGroup::Id("BulbGroupIdB"))}}};
  const DiscreteValueRule::DiscreteValue kDiscreteValueWithEmptyKeyRelatedUniqueIds = DiscreteValueRule::DiscreteValue{
      Rule::State::kStrict, api::test::CreateEmptyRelatedRules(), kEmptyKeyRelatedUniqueIds, "rule_state_value"};
  EXPECT_THROW(DiscreteValueRule(kId, kTypeId, kZone, {kDiscreteValueWithEmptyKeyRelatedUniqueIds}),
               maliput::common::assertion_error);

  // Empty discrete values.
  EXPECT_THROW(DiscreteValueRule(kId, kTypeId, kZone, {} /* discrete_values */), maliput::common::assertion_error);
}

// Evaluates DiscreteValueRule accessors.
TEST_F(RuleTest, DiscreteValueRuleAccessors) {
  const std::vector<DiscreteValueRule::DiscreteValue> kDiscreteValues{
      DiscreteValueRule::DiscreteValue{Rule::State::kStrict, api::test::CreateEmptyRelatedRules(),
                                       api::test::CreateEmptyRelatedUniqueIds(), "rule_state_value_1"},
      DiscreteValueRule::DiscreteValue{Rule::State::kBestEffort, api::test::CreateNonEmptyRelatedRules(),
                                       api::test::CreateNonEmptyRelatedUniqueIds(), "rule_state_value_2"}};

  const DiscreteValueRule dut(kId, kTypeId, kZone, kDiscreteValues);

  EXPECT_EQ(dut.id(), kId);
  EXPECT_EQ(dut.type_id(), kTypeId);
  EXPECT_TRUE(MALIPUT_REGIONS_IS_EQUAL(dut.zone(), kZone));
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut.states(), kDiscreteValues));
}

// Evaluates the equal and not equal operator overloads for
// DiscreteValueRule::DiscreteValue.
GTEST_TEST(DiscreteValueTest, EqualOperator) {
  const DiscreteValueRule::DiscreteValue discrete_value_1 = DiscreteValueRule::DiscreteValue{
      Rule::State::kStrict, api::test::CreateEmptyRelatedRules(), api::test::CreateEmptyRelatedUniqueIds(), "value_1"};
  const DiscreteValueRule::DiscreteValue discrete_value_2 = DiscreteValueRule::DiscreteValue{
      Rule::State::kStrict, api::test::CreateEmptyRelatedRules(), api::test::CreateEmptyRelatedUniqueIds(), "value_2"};
  const DiscreteValueRule::DiscreteValue discrete_value_3 =
      DiscreteValueRule::DiscreteValue{Rule::State::kBestEffort, api::test::CreateEmptyRelatedRules(),
                                       api::test::CreateEmptyRelatedUniqueIds(), "value_1"};
  const DiscreteValueRule::DiscreteValue discrete_value_4 =
      DiscreteValueRule::DiscreteValue{Rule::State::kBestEffort, api::test::CreateNonEmptyRelatedRules(),
                                       api::test::CreateNonEmptyRelatedUniqueIds(), "value_4"};

  // `related_rules` and `related_unique_ids` are empty.
  EXPECT_TRUE(discrete_value_1 == discrete_value_1);
  EXPECT_FALSE(discrete_value_1 != discrete_value_1);
  // `value` is different.
  EXPECT_TRUE(discrete_value_1 != discrete_value_2);
  EXPECT_FALSE(discrete_value_1 == discrete_value_2);
  // `severity` is different.
  EXPECT_TRUE(discrete_value_1 != discrete_value_3);
  EXPECT_FALSE(discrete_value_1 == discrete_value_3);
  // `related_rules` and `related_unique_ids` are not empty.
  EXPECT_TRUE(discrete_value_4 == discrete_value_4);
  EXPECT_FALSE(discrete_value_4 != discrete_value_4);
}

}  // namespace
}  // namespace rules
}  // namespace api
}  // namespace maliput
