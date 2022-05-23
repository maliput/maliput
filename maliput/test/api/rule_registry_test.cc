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
#include "maliput/api/rules/rule_registry.h"

#include <gtest/gtest.h>

#include "maliput/common/assertion_error.h"
#include "maliput/test_utilities/mock.h"
#include "maliput/test_utilities/regions_test_utilities.h"
#include "maliput/test_utilities/rules_compare.h"
#include "maliput/test_utilities/rules_test_utilities.h"

namespace maliput {
namespace api {
namespace rules {
namespace test {

// Evaluates queries to an empty RuleRegistry.
GTEST_TEST(EmptyRuleRegistry, AccessorsTest) {
  const RuleRegistry dut;

  EXPECT_TRUE(dut.RangeValueRuleTypes().empty());
  EXPECT_TRUE(dut.DiscreteValueRuleTypes().empty());

  const std::optional<RuleRegistry::QueryResult> result =
      dut.GetPossibleStatesOfRuleType(Rule::TypeId("any_rule_type"));
  EXPECT_FALSE(result.has_value());
}

// Evaluates queries after registering RangeValueRule types.
GTEST_TEST(RegisterRangeValueRule, RegisterAndQueryTest) {
  const Rule::TypeId kTypeA("RangeValueRuleTypeA");
  const Rule::TypeId kTypeB("RangeValueRuleTypeB");
  const Rule::TypeId kTypeC("RangeValueRuleTypeC");
  const RangeValueRule::Range kRangeA{Rule::State::kStrict,
                                      api::test::CreateEmptyRelatedRules(),
                                      api::test::CreateEmptyRelatedUniqueIds(),
                                      "range_description_a",
                                      123.,
                                      456.};
  const RangeValueRule::Range kRangeB{Rule::State::kBestEffort,
                                      api::test::CreateEmptyRelatedRules(),
                                      api::test::CreateEmptyRelatedUniqueIds(),
                                      "range_description_b",
                                      456.,
                                      789.};

  RuleRegistry dut;
  // Registers RangeValueRule types.
  EXPECT_NO_THROW(dut.RegisterRangeValueRule(kTypeA, {kRangeA, kRangeB}));
  EXPECT_NO_THROW(dut.RegisterRangeValueRule(kTypeB, {kRangeA}));
  // Throws because of duplicated type ID.
  EXPECT_THROW(dut.RegisterRangeValueRule(kTypeB, {kRangeA}), maliput::common::assertion_error);
  EXPECT_THROW(dut.RegisterDiscreteValueRule(
                   kTypeB, {DiscreteValueRule::DiscreteValue{Rule::State::kStrict, api::test::CreateEmptyRelatedRules(),
                                                             api::test::CreateEmptyRelatedUniqueIds(), "SomeValue"}}),
               maliput::common::assertion_error);
  // Throws because of empty range vector.
  EXPECT_THROW(dut.RegisterRangeValueRule(kTypeC, {} /* ranges */), maliput::common::assertion_error);
  // Throws because of duplicated ranges.
  EXPECT_THROW(dut.RegisterRangeValueRule(kTypeC, {kRangeA, kRangeA}), maliput::common::assertion_error);

  EXPECT_TRUE(dut.DiscreteValueRuleTypes().empty());

  const std::map<Rule::TypeId, RuleRegistry::QueryResult::Ranges> kExpectedRuleTypes{{kTypeA, {kRangeA, kRangeB}},
                                                                                     {kTypeB, {kRangeA}}};

  const std::map<Rule::TypeId, RuleRegistry::QueryResult::Ranges> range_value_rule_types = dut.RangeValueRuleTypes();
  EXPECT_EQ(range_value_rule_types.size(), kExpectedRuleTypes.size());
  for (const auto& rule_type : kExpectedRuleTypes) {
    const auto found_rule_values = range_value_rule_types.find(rule_type.first);
    EXPECT_NE(found_rule_values, range_value_rule_types.end());
    EXPECT_EQ(found_rule_values->second.size(), rule_type.second.size());
    for (const RangeValueRule::Range& range : found_rule_values->second) {
      EXPECT_NE(std::find(rule_type.second.begin(), rule_type.second.end(), range), rule_type.second.end());
    }
  }

  // Finds each type.
  {
    const std::optional<RuleRegistry::QueryResult> result = dut.GetPossibleStatesOfRuleType(kTypeA);
    EXPECT_TRUE(result.has_value());
    EXPECT_EQ(result->type_id, kTypeA);
    const auto ranges_ptr = std::get_if<RuleRegistry::QueryResult::Ranges>(&result->rule_values);
    EXPECT_NE(ranges_ptr, nullptr);
    EXPECT_EQ(static_cast<int>(ranges_ptr->size()), 2);
    EXPECT_EQ(ranges_ptr->at(0), kRangeA);
    EXPECT_EQ(ranges_ptr->at(1), kRangeB);
    EXPECT_EQ(std::get_if<RuleRegistry::QueryResult::DiscreteValues>(&result->rule_values), nullptr);
  }
  {
    const std::optional<RuleRegistry::QueryResult> result = dut.GetPossibleStatesOfRuleType(kTypeB);
    EXPECT_TRUE(result.has_value());
    EXPECT_EQ(result->type_id, kTypeB);
    const auto ranges_ptr = std::get_if<RuleRegistry::QueryResult::Ranges>(&result->rule_values);
    EXPECT_NE(ranges_ptr, nullptr);
    EXPECT_EQ(static_cast<int>(ranges_ptr->size()), 1);
    EXPECT_EQ(ranges_ptr->at(0), kRangeA);
    EXPECT_EQ(std::get_if<RuleRegistry::QueryResult::DiscreteValues>(&result->rule_values), nullptr);
  }
  {
    const std::optional<RuleRegistry::QueryResult> result =
        dut.GetPossibleStatesOfRuleType(Rule::TypeId("any_rule_type"));
    EXPECT_FALSE(result.has_value());
  }
}

// Evaluates queries after registering DiscreteValueRule types.
GTEST_TEST(RegisterDiscreteValueRule, RegisterAndQueryTest) {
  const Rule::TypeId kTypeA("DiscreteValueTypeA");
  const RuleRegistry::QueryResult::DiscreteValues kValuesA{
      DiscreteValueRule::DiscreteValue{Rule::State::kStrict, api::test::CreateEmptyRelatedRules(),
                                       api::test::CreateEmptyRelatedUniqueIds(), "ValueA1"},
      DiscreteValueRule::DiscreteValue{Rule::State::kBestEffort, api::test::CreateEmptyRelatedRules(),
                                       api::test::CreateEmptyRelatedUniqueIds(), "ValueA2"}};
  const Rule::TypeId kTypeB("DiscreteValueRuleTypeB");
  const RuleRegistry::QueryResult::DiscreteValues kValuesB{
      DiscreteValueRule::DiscreteValue{Rule::State::kStrict, api::test::CreateEmptyRelatedRules(),
                                       api::test::CreateEmptyRelatedUniqueIds(), "ValueB1"},
      DiscreteValueRule::DiscreteValue{Rule::State::kStrict, api::test::CreateEmptyRelatedRules(),
                                       api::test::CreateEmptyRelatedUniqueIds(), "ValueB2"},
      DiscreteValueRule::DiscreteValue{Rule::State::kBestEffort, api::test::CreateEmptyRelatedRules(),
                                       api::test::CreateEmptyRelatedUniqueIds(), "ValueB3"}};
  const RangeValueRule::Range kRange{Rule::State::kStrict,
                                     api::test::CreateEmptyRelatedRules(),
                                     api::test::CreateEmptyRelatedUniqueIds(),
                                     "range_description_a",
                                     123.,
                                     456.};

  RuleRegistry dut;
  // Registers DiscreteValueRule types.
  EXPECT_NO_THROW(dut.RegisterDiscreteValueRule(kTypeA, kValuesA));
  EXPECT_NO_THROW(dut.RegisterDiscreteValueRule(kTypeB, kValuesB));
  // Throws because of duplicated type ID.
  EXPECT_THROW(dut.RegisterDiscreteValueRule(
                   kTypeB, {DiscreteValueRule::DiscreteValue{Rule::State::kStrict, api::test::CreateEmptyRelatedRules(),
                                                             api::test::CreateEmptyRelatedUniqueIds(), "SomeValue"}}),
               maliput::common::assertion_error);
  EXPECT_THROW(dut.RegisterRangeValueRule(kTypeB, {kRange}), maliput::common::assertion_error);
  // Throws because of empty vector.
  EXPECT_THROW(dut.RegisterDiscreteValueRule(Rule::TypeId("SomeRuleType"), {}), maliput::common::assertion_error);

  EXPECT_TRUE(dut.RangeValueRuleTypes().empty());

  const std::map<Rule::TypeId, RuleRegistry::QueryResult::DiscreteValues> kExpectedRuleTypes{{kTypeA, kValuesA},
                                                                                             {kTypeB, kValuesB}};
  const std::map<Rule::TypeId, RuleRegistry::QueryResult::DiscreteValues> discrete_value_rule_types =
      dut.DiscreteValueRuleTypes();
  EXPECT_EQ(discrete_value_rule_types.size(), kExpectedRuleTypes.size());
  for (const auto& rule_values : kExpectedRuleTypes) {
    const auto found_rule_values = discrete_value_rule_types.find(rule_values.first);
    EXPECT_NE(found_rule_values, discrete_value_rule_types.end());
    EXPECT_EQ(found_rule_values->second.size(), rule_values.second.size());
    for (const DiscreteValueRule::DiscreteValue& value : found_rule_values->second) {
      EXPECT_NE(std::find(rule_values.second.begin(), rule_values.second.end(), value), rule_values.second.end());
    }
  }

  // Finds each type.
  {
    const std::optional<RuleRegistry::QueryResult> result = dut.GetPossibleStatesOfRuleType(kTypeA);
    EXPECT_TRUE(result.has_value());
    EXPECT_EQ(std::get_if<RuleRegistry::QueryResult::Ranges>(&result->rule_values), nullptr);
    EXPECT_EQ(result->type_id, kTypeA);
    const auto discrete_values_ptr = std::get_if<RuleRegistry::QueryResult::DiscreteValues>(&result->rule_values);
    EXPECT_NE(discrete_values_ptr, nullptr);
    EXPECT_EQ(discrete_values_ptr->size(), kValuesA.size());
    for (const DiscreteValueRule::DiscreteValue& value : *discrete_values_ptr) {
      EXPECT_NE(std::find(kValuesA.begin(), kValuesA.end(), value), kValuesA.end());
    }
  }
  {
    const std::optional<RuleRegistry::QueryResult> result = dut.GetPossibleStatesOfRuleType(kTypeB);
    EXPECT_TRUE(result.has_value());
    EXPECT_EQ(std::get_if<RuleRegistry::QueryResult::Ranges>(&result->rule_values), nullptr);
    EXPECT_EQ(result->type_id, kTypeB);
    const auto discrete_values_ptr = std::get_if<RuleRegistry::QueryResult::DiscreteValues>(&result->rule_values);
    EXPECT_NE(discrete_values_ptr, nullptr);
    EXPECT_EQ(discrete_values_ptr->size(), kValuesB.size());
    for (const DiscreteValueRule::DiscreteValue& value : *discrete_values_ptr) {
      EXPECT_NE(std::find(kValuesB.begin(), kValuesB.end(), value), kValuesB.end());
    }
  }
  {
    const std::optional<RuleRegistry::QueryResult> result =
        dut.GetPossibleStatesOfRuleType(Rule::TypeId("any_rule_type"));
    EXPECT_FALSE(result.has_value());
  }
}

// Registers RangeValueRules and DiscreteValueRules, then builds rules.
GTEST_TEST(RegisterAndBuildTest, RegisterAndBuild) {
  const Rule::TypeId kRangeValueRuleType("RangeValueRuleType");
  const Rule::Id kRangeRuleId("RangeValueRuleType/RangeRuleId");
  const LaneSRoute kZone({LaneSRange(LaneId("LaneId"), SRange(10., 20.))});
  const RangeValueRule::Range kRangeA{Rule::State::kStrict,
                                      api::test::CreateEmptyRelatedRules(),
                                      api::test::CreateEmptyRelatedUniqueIds(),
                                      "range_description",
                                      123.,
                                      456.};
  const RangeValueRule::Range kRangeB{Rule::State::kStrict,
                                      api::test::CreateNonEmptyRelatedRules(),
                                      api::test::CreateEmptyRelatedUniqueIds(),
                                      "range_description",
                                      123.,
                                      456.};
  const RangeValueRule::Range kUnregisteredRange{Rule::State::kBestEffort,
                                                 api::test::CreateEmptyRelatedRules(),
                                                 api::test::CreateEmptyRelatedUniqueIds(),
                                                 "range_description",
                                                 456.,
                                                 789.};
  const Rule::TypeId kDiscreteValueRuleType("DiscreteValueType");
  const Rule::Id kDiscreteValueRuleId("DiscreteValueType/DiscreteValueRuleId");
  const RuleRegistry::QueryResult::DiscreteValues kDiscreteValues{
      DiscreteValueRule::DiscreteValue{Rule::State::kStrict, api::test::CreateEmptyRelatedRules(),
                                       api::test::CreateEmptyRelatedUniqueIds(), "Value1"},
      DiscreteValueRule::DiscreteValue{Rule::State::kStrict, api::test::CreateEmptyRelatedRules(),
                                       api::test::CreateEmptyRelatedUniqueIds(), "Value2"},
      DiscreteValueRule::DiscreteValue{Rule::State::kStrict, api::test::CreateEmptyRelatedRules(),
                                       api::test::CreateEmptyRelatedUniqueIds(), "Value3"}};

  const Rule::TypeId kUnregisteredRuleType("UnregisteredRuleType");
  const DiscreteValueRule::DiscreteValue kUnregisteredDiscreteValue{
      Rule::State::kStrict, api::test::CreateEmptyRelatedRules(), api::test::CreateEmptyRelatedUniqueIds(), "Value4"};

  RuleRegistry dut;

  dut.RegisterRangeValueRule(kRangeValueRuleType, {kRangeA});
  dut.RegisterDiscreteValueRule(kDiscreteValueRuleType, kDiscreteValues);

  // Builds and evaluates a RangeValueRule.
  RangeValueRule range_value_rule = dut.BuildRangeValueRule(kRangeRuleId, kRangeValueRuleType, kZone, {kRangeA});
  EXPECT_EQ(range_value_rule.id(), kRangeRuleId);
  EXPECT_EQ(range_value_rule.type_id(), kRangeValueRuleType);
  EXPECT_TRUE(MALIPUT_REGIONS_IS_EQUAL(range_value_rule.zone(), kZone));
  EXPECT_TRUE(MALIPUT_IS_EQUAL(range_value_rule.states(), {kRangeA}));

  // Builds and evaluates a RangeValueRule of the same type but with non-empty RelatedRules.
  range_value_rule = dut.BuildRangeValueRule(kRangeRuleId, kRangeValueRuleType, kZone, {kRangeB});
  EXPECT_EQ(range_value_rule.id(), kRangeRuleId);
  EXPECT_EQ(range_value_rule.type_id(), kRangeValueRuleType);
  EXPECT_TRUE(MALIPUT_REGIONS_IS_EQUAL(range_value_rule.zone(), kZone));
  EXPECT_TRUE(MALIPUT_IS_EQUAL(range_value_rule.states(), {kRangeB}));

  // Unregistered type.
  EXPECT_THROW(dut.BuildRangeValueRule(Rule::Id("RuleId"), kUnregisteredRuleType, kZone, {kRangeA}),
               maliput::common::assertion_error);
  // Unregistered range.
  EXPECT_THROW(dut.BuildRangeValueRule(Rule::Id("RuleId"), kUnregisteredRuleType, kZone, {kUnregisteredRange}),
               maliput::common::assertion_error);

  // Builds and evaluates a discrete value based rule.
  const RuleRegistry::QueryResult::DiscreteValues kExpectedDiscreteValues{
      DiscreteValueRule::DiscreteValue{Rule::State::kStrict, api::test::CreateEmptyRelatedRules(),
                                       api::test::CreateEmptyRelatedUniqueIds(), "Value1"},
      DiscreteValueRule::DiscreteValue{Rule::State::kStrict, api::test::CreateNonEmptyRelatedRules(),
                                       api::test::CreateEmptyRelatedUniqueIds(), "Value3"}};
  DiscreteValueRule discrete_value_rule =
      dut.BuildDiscreteValueRule(kDiscreteValueRuleId, kDiscreteValueRuleType, kZone, kExpectedDiscreteValues);
  EXPECT_EQ(discrete_value_rule.id(), kDiscreteValueRuleId);
  EXPECT_EQ(discrete_value_rule.type_id(), kDiscreteValueRuleType);
  EXPECT_TRUE(MALIPUT_REGIONS_IS_EQUAL(discrete_value_rule.zone(), kZone));
  EXPECT_TRUE(MALIPUT_IS_EQUAL(discrete_value_rule.states(), kExpectedDiscreteValues));

  // Unregistered type.
  EXPECT_THROW(dut.BuildDiscreteValueRule(Rule::Id("RuleId"), kUnregisteredRuleType, kZone, kExpectedDiscreteValues),
               maliput::common::assertion_error);
  // Unregistered discrete value for the type.
  EXPECT_THROW(
      dut.BuildDiscreteValueRule(kDiscreteValueRuleId, kDiscreteValueRuleType, kZone, {kUnregisteredDiscreteValue}),
      maliput::common::assertion_error);
}

}  // namespace test
}  // namespace rules
}  // namespace api
}  // namespace maliput
