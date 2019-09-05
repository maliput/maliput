#include "maliput/base/manual_discrete_value_rule_state_provider.h"

#include <stdexcept>

#include <gtest/gtest.h>

#include "maliput/test_utilities/rules_compare.h"
#include "maliput/test_utilities/rules_test_utilities.h"

namespace maliput {
namespace test {
namespace {

using maliput::api::rules::DiscreteValueRule;
using maliput::api::rules::DiscreteValueRuleStateProvider;
using maliput::api::rules::Rule;

GTEST_TEST(ManualDiscreteRuleStateProviderTest, BasicTest) {
  ManualDiscreteValueRuleStateProvider dut;

  const Rule::Id kRuleId("rule_type/rule_id");
  const DiscreteValueRule::DiscreteValue kState = maliput::api::rules::MakeDiscreteValue(Rule::State::kStrict, "value");

  // No rule with specified ID exists.
  EXPECT_FALSE(dut.GetState(kRuleId).has_value());
  EXPECT_THROW(dut.SetState(kRuleId, kState), std::out_of_range);

  // Adds a state and evaluates the result.
  dut.AddState(kRuleId, kState);
  drake::optional<DiscreteValueRuleStateProvider::StateResult> result = dut.GetState(kRuleId);
  EXPECT_TRUE(result.has_value());
  EXPECT_TRUE(MALIPUT_IS_EQUAL(result->state_value, kState));
  EXPECT_EQ(result->next, drake::nullopt);

  // Attempting to add duplicate state.
  EXPECT_THROW(dut.AddState(kRuleId, kState), std::logic_error);
}

}  // namespace
}  // namespace test
}  // namespace maliput
