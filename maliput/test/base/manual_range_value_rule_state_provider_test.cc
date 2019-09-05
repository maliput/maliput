/* clang-format off to disable clang-format-includes */
#include "maliput/base/manual_range_value_rule_state_provider.h"
/* clang-format on */

#include <stdexcept>

#include <gtest/gtest.h>

#include "maliput/test_utilities/rules_compare.h"
#include "maliput/test_utilities/rules_test_utilities.h"

namespace maliput {
namespace test {
namespace {

using maliput::api::rules::RangeValueRule;
using maliput::api::rules::RangeValueRuleStateProvider;
using maliput::api::rules::Rule;

GTEST_TEST(ManualRangeRuleStateProviderTest, BasicTest) {
  ManualRangeValueRuleStateProvider dut;

  const Rule::Id kRuleId("rule_type/rule_id");
  const RangeValueRule::Range kRangeState =
      maliput::api::rules::MakeRange(Rule::State::kStrict, "range_description", 123. /* min */, 456. /* max */);

  // No rule with specified ID exists.
  EXPECT_FALSE(dut.GetState(kRuleId).has_value());
  EXPECT_THROW(dut.SetState(kRuleId, kRangeState), std::out_of_range);

  // Adds a state and evaluates the result.
  dut.AddState(kRuleId, kRangeState);
  drake::optional<RangeValueRuleStateProvider::StateResult> result = dut.GetState(kRuleId);
  EXPECT_TRUE(result.has_value());
  EXPECT_TRUE(MALIPUT_IS_EQUAL(result->state_range, kRangeState));
  EXPECT_EQ(result->next, drake::nullopt);

  // Attempting to add duplicate state.
  EXPECT_THROW(dut.AddState(kRuleId, kRangeState), std::logic_error);
}

}  // namespace
}  // namespace test
}  // namespace maliput
