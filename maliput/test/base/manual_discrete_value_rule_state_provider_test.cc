#include "maliput/base/manual_discrete_value_rule_state_provider.h"

#include <stdexcept>

#include <gtest/gtest.h>

#include "maliput/api/regions.h"
#include "maliput/api/rules/discrete_value_rule.h"
#include "maliput/api/rules/rule.h"
#include "maliput/base/manual_rulebook.h"
#include "maliput/base/rule_registry.h"
#include "maliput/base/rule_tools.h"
#include "maliput/common/assertion_error.h"
#include "maliput/test_utilities/mock.h"
#include "maliput/test_utilities/rules_compare.h"
#include "maliput/test_utilities/rules_test_utilities.h"

namespace maliput {
namespace test {
namespace {

using api::LaneId;
using api::LaneSRange;
using api::LaneSRoute;
using api::rules::DiscreteValueRule;
using api::rules::DiscreteValueRuleStateProvider;
using api::rules::MakeDiscreteValue;
using api::rules::RoadRulebook;
using api::rules::Rule;

class ManualDiscreteRuleStateProviderTest : public ::testing::Test {
 protected:
  const Rule::Id kRuleId{"dvrt/dvr_id"};
  const Rule::Id kUnknownRuleId{"dvrt/unknown_id"};
  const DiscreteValueRule::DiscreteValue kStateA{
      MakeDiscreteValue(Rule::State::kStrict, api::test::CreateEmptyRelatedRules(), "value1")};
  const DiscreteValueRule::DiscreteValue kStateB{
      MakeDiscreteValue(Rule::State::kStrict, api::test::CreateEmptyRelatedRules(), "value2")};
  const DiscreteValueRule::DiscreteValue kInvalidState{
      MakeDiscreteValue(Rule::State::kStrict, api::test::CreateEmptyRelatedRules(), "invalid_state")};
  const double kDurationUntil{10.};

  void SetUp() override {
    const maliput::api::test::RoadRulebookBuildFlags kRulebookBuildFlags{
        false /* add_right_of_way */, {} /* right_of_way_build_flags */,  false /* add_direction_usage */,
        false /* add_speed_limit */,  true /* add_discrete_value_rule */, false /* add_range_value_rule */};
    road_rulebook_ = api::test::CreateRoadRulebook(kRulebookBuildFlags);
  }

  std::unique_ptr<RoadRulebook> road_rulebook_;
};

TEST_F(ManualDiscreteRuleStateProviderTest, ConstructorConstraints) {
  EXPECT_THROW(ManualDiscreteValueRuleStateProvider(nullptr), maliput::common::assertion_error);
  EXPECT_NO_THROW(ManualDiscreteValueRuleStateProvider(road_rulebook_.get()));
}

TEST_F(ManualDiscreteRuleStateProviderTest, SetStateTest) {
  ManualDiscreteValueRuleStateProvider dut(road_rulebook_.get());

  // Tries to set the state to an unknown Rule::Id in `rulebook_`.
  EXPECT_THROW(dut.SetState(kUnknownRuleId, kStateA, {}, {}), std::out_of_range);
  // Tries to set an invalid state to the rule.
  EXPECT_THROW(dut.SetState(kRuleId, kInvalidState, {}, {}), maliput::common::assertion_error);
  // Tries to set an invalid next state to the rule.
  EXPECT_THROW(dut.SetState(kRuleId, kStateA, {kInvalidState}, {}), maliput::common::assertion_error);
  // Tries to set a valid next state with a negative duration.
  EXPECT_THROW(dut.SetState(kRuleId, kStateA, {kStateA}, {-kDurationUntil}), maliput::common::assertion_error);
  // Tries to set a nullopt next state with duration.
  EXPECT_THROW(dut.SetState(kRuleId, kStateA, {}, {kDurationUntil}), maliput::common::assertion_error);

  // Sets a valid state, next state and duration until.
  EXPECT_NO_THROW(dut.SetState(kRuleId, kStateA, {kStateB}, {kDurationUntil}));
  const drake::optional<DiscreteValueRuleStateProvider::StateResult> result = dut.GetState(kRuleId);
  EXPECT_TRUE(result.has_value());
  EXPECT_TRUE(MALIPUT_IS_EQUAL(result->state, kStateA));
  EXPECT_TRUE(result->next.has_value());
  EXPECT_TRUE(MALIPUT_IS_EQUAL(result->next->state, kStateB));
  EXPECT_TRUE(result->next->duration_until.has_value());
  EXPECT_EQ(result->next->duration_until.value(), kDurationUntil);
}

api::rules::Rule::RelatedRules CreateRightOfWayRelatedRule() {
  return std::map<std::string, std::vector<api::rules::Rule::Id>>{
      {VehicleStopInZoneBehaviorRuleTypeId().string(),
       {api::rules::Rule::Id{VehicleStopInZoneBehaviorRuleTypeId().string() + "/" + "test_id"}}},
      {RightOfWayYieldGroup(),
       {api::rules::Rule::Id{RightOfWayRuleTypeId().string() + "/" + "test_yield_id_1"},
        api::rules::Rule::Id{RightOfWayRuleTypeId().string() + "/" + "test_yield_id_2"}}},
  };
}

class GetCurrentYieldGroupTest : public ::testing::Test {
 protected:
  const Rule::Id kRuleId{RightOfWayRuleTypeId().string() + "/right_of_way_rule_id"};
  const Rule::TypeId kTypeId{RightOfWayRuleTypeId()};
  const LaneSRoute kLaneSRoute{LaneSRoute{{LaneSRange{LaneId{"lane_id"}, {0., 10.}}}}};
  const DiscreteValueRule::DiscreteValue kStateDiscreteValue{
      MakeDiscreteValue(Rule::State::kStrict, CreateRightOfWayRelatedRule(), "test_value_state")};
  const std::vector<Rule::Id> expected_yield_group{CreateRightOfWayRelatedRule().at(RightOfWayYieldGroup())};

  void SetUp() override {
    road_rulebook_->AddRule(DiscreteValueRule{kRuleId, kTypeId, kLaneSRoute, {kStateDiscreteValue}});
  }

  std::unique_ptr<ManualRulebook> road_rulebook_ = std::make_unique<ManualRulebook>();
  //--CHECK. What is the difference with:
  // std::unique_ptr<ManualRulebook> road_rulebook_;
};

// TODO docstring
TEST_F(GetCurrentYieldGroupTest, GetCurrentYieldGroup) {
  maliput::ManualDiscreteValueRuleStateProvider discrete_value_rule_state_provider{road_rulebook_.get()};
  discrete_value_rule_state_provider.SetState(kRuleId, kStateDiscreteValue, drake::nullopt,
                                              drake::nullopt);  // --CHECK. No nextstate?
  const DiscreteValueRule discrete_value_rule{road_rulebook_->GetDiscreteValueRule(kRuleId)};
  std::vector<Rule::Id> dut{maliput::GetCurrentYieldGroup(discrete_value_rule, &discrete_value_rule_state_provider)};

  EXPECT_EQ(dut.size(), expected_yield_group.size());
  for (const auto& expected_yield_id : expected_yield_group) {
    EXPECT_NE(std::find(dut.begin(), dut.end(), expected_yield_id), dut.end());
  }
}

}  // namespace
}  // namespace test
}  // namespace maliput
