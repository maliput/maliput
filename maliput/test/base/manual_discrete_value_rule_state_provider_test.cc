#include "maliput/base/manual_discrete_value_rule_state_provider.h"

#include <stdexcept>

#include <gtest/gtest.h>

#include "maliput/api/rules/discrete_value_rule.h"
#include "maliput/api/rules/rule.h"
#include "maliput/common/assertion_error.h"
#include "maliput/test_utilities/mock.h"
#include "maliput/test_utilities/rules_compare.h"
#include "maliput/test_utilities/rules_test_utilities.h"

namespace maliput {
namespace test {
namespace {

using maliput::api::rules::DiscreteValueRule;
using maliput::api::rules::DiscreteValueRuleStateProvider;
using maliput::api::rules::MakeDiscreteValue;
using maliput::api::rules::RoadRulebook;
using maliput::api::rules::Rule;

class ManualDiscreteRuleStateProviderTest : public ::testing::Test {
 protected:
  const Rule::Id kRuleId{"dvrt/dvr_id"};
  const Rule::Id kUnknownRuleId{"dvrt/unknown_id"};
  const DiscreteValueRule::DiscreteValue kStateA{
      MakeDiscreteValue(Rule::State::kStrict, maliput::api::test::CreateEmptyRelatedRules(), "value1")};
  const DiscreteValueRule::DiscreteValue kStateB{
      MakeDiscreteValue(Rule::State::kStrict, maliput::api::test::CreateEmptyRelatedRules(), "value2")};
  const DiscreteValueRule::DiscreteValue kInvalidState{
      MakeDiscreteValue(Rule::State::kStrict, maliput::api::test::CreateEmptyRelatedRules(), "invalid_state")};

  void SetUp() override {
    const maliput::api::test::RoadRulebookBuildFlags kRulebookBuildFlags{
        false /* add_right_of_way */, {} /* right_of_way_build_flags */,  false /* add_direction_usage */,
        false /* add_speed_limit */,  true /* add_discrete_value_rule */, false /* add_range_value_rule */};
    road_rulebook_ = maliput::api::test::CreateRoadRulebook(kRulebookBuildFlags);
  }

  std::unique_ptr<RoadRulebook> road_rulebook_;
};

TEST_F(ManualDiscreteRuleStateProviderTest, ConstructorConstraints) {
  EXPECT_THROW(ManualDiscreteValueRuleStateProvider(nullptr), maliput::common::assertion_error);
  EXPECT_NO_THROW(ManualDiscreteValueRuleStateProvider(road_rulebook_.get()));
}

TEST_F(ManualDiscreteRuleStateProviderTest, BasicTest) {
  ManualDiscreteValueRuleStateProvider dut(road_rulebook_.get());

  // No rule with specified ID exists.
  EXPECT_FALSE(dut.GetState(kRuleId).has_value());
  EXPECT_THROW(dut.SetState(kRuleId, kStateA, {}), std::out_of_range);

  // Tries to register a state to an invalid Rule::Id.
  EXPECT_THROW(dut.Register(kUnknownRuleId, kStateA), std::out_of_range);
  // Tries to register an invalid state to a valid Rule.
  EXPECT_THROW(dut.Register(kRuleId, kInvalidState), maliput::common::assertion_error);
  // Registers a valid state.
  EXPECT_NO_THROW(dut.Register(kRuleId, kStateA));
  // Tries to register a state to an already registered rule.
  EXPECT_THROW(dut.Register(kRuleId, kStateB), std::logic_error);

  // Adds a state and evaluates the result.
  drake::optional<DiscreteValueRuleStateProvider::StateResult> result = dut.GetState(kRuleId);
  EXPECT_TRUE(result.has_value());
  EXPECT_TRUE(MALIPUT_IS_EQUAL(result->value_state, kStateA));
  EXPECT_EQ(result->next, drake::nullopt);

  // Tries to set the state to an unknown Rule::Id in `rulebook_`.
  EXPECT_THROW(dut.SetState(kUnknownRuleId, kStateA, {}), std::out_of_range);
  // Tries to set an invalid state to the rule.
  EXPECT_THROW(dut.SetState(kRuleId, kInvalidState, {}), maliput::common::assertion_error);
  // Tries to set an invalid next state to the rule.
  EXPECT_THROW(dut.SetState(kRuleId, kStateA, {kInvalidState}), maliput::common::assertion_error);
  // Sets a valid next state to the rule.
  EXPECT_NO_THROW(dut.SetState(kRuleId, kStateA, {kStateB}));

  // Evaluates the result.
  result = dut.GetState(kRuleId);
  EXPECT_TRUE(result.has_value());
  EXPECT_TRUE(MALIPUT_IS_EQUAL(result->value_state, kStateA));
  EXPECT_TRUE(result->next.has_value());
  EXPECT_TRUE(MALIPUT_IS_EQUAL(result->next->value_state, kStateB));
  EXPECT_FALSE(result->next->duration_until.has_value());
}

}  // namespace
}  // namespace test
}  // namespace maliput
