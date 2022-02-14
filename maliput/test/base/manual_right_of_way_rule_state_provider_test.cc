#include "maliput/base/manual_right_of_way_rule_state_provider.h"

#include <stdexcept>

#include <gtest/gtest.h>

#include "maliput/test_utilities/rules_right_of_way_compare.h"

namespace maliput {
namespace test {
namespace {

using api::rules::RightOfWayRule;
using api::rules::RightOfWayRuleStateProvider;

GTEST_TEST(ManualRightOfWayRuleStateProviderTest, BasicTest) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  ManualRightOfWayRuleStateProvider dut;
  const RightOfWayRule::Id kRuleId("foo");
  const RightOfWayRule::State::Id kStateId("bar");

  // No rule with specified ID exists.
  EXPECT_FALSE(dut.GetState(kRuleId).has_value());
  EXPECT_THROW(dut.SetState(kRuleId, kStateId), std::out_of_range);

  dut.AddState(kRuleId, kStateId);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut.GetState(kRuleId).value(),
                               (RightOfWayRuleStateProvider::RightOfWayResult{kStateId, std::nullopt})));

  // Attempting to add duplicate state.
  EXPECT_THROW(dut.AddState(kRuleId, kStateId), std::logic_error);

  const RightOfWayRule::State::Id kOtherStateId("baz");
  dut.SetState(kRuleId, kOtherStateId);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut.GetState(kRuleId).value(),
                               (RightOfWayRuleStateProvider::RightOfWayResult{kOtherStateId, std::nullopt})));
#pragma GCC diagnostic pop
}

}  // namespace
}  // namespace test
}  // namespace maliput
