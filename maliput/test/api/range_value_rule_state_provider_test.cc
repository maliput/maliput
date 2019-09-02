#include "maliput/api/rules/range_value_rule_state_provider.h"

#include <gtest/gtest.h>

namespace maliput {
namespace api {
namespace rules {
namespace {

// Mock class to evaluate RangeValueRuleStateProvider interface.
class MockRangeValueRuleStateProvider : public RangeValueRuleStateProvider {
 public:
  static const Rule::Id kRuleId;

 private:
  drake::optional<StateResult> DoGetState(const Rule::Id& id) const override {
    if (id == kRuleId) {
      return StateResult{RangeValueRule::Range{"current_range_description", 56. /* min */, 78. /* max*/},
                         StateResult::Next{RangeValueRule::Range{"next_range_description", 12. /* min */, 34. /* max*/},
                                           {123.456} /* duration */}};
    }
    return {};
  }
};

const Rule::Id MockRangeValueRuleStateProvider::kRuleId{"RuleId"};

GTEST_TEST(RangeValueRuleStateProviderTest, ExerciseInterface) {
  const MockRangeValueRuleStateProvider dut;
  const drake::optional<RangeValueRuleStateProvider::StateResult> result =
      dut.GetState(MockRangeValueRuleStateProvider::kRuleId);

  EXPECT_TRUE(result.has_value());
  EXPECT_EQ(result->state_range.description, "current_range_description");
  EXPECT_EQ(result->state_range.min, 56.);
  EXPECT_EQ(result->state_range.max, 78.);
  EXPECT_TRUE(result->next.has_value());
  EXPECT_EQ(result->next->state_range.description, "next_range_description");
  EXPECT_EQ(result->next->state_range.min, 12.);
  EXPECT_EQ(result->next->state_range.max, 34.);
  EXPECT_TRUE(result->next->duration_until.has_value());
  EXPECT_EQ(result->next->duration_until.value(), 123.456);

  EXPECT_FALSE(dut.GetState(Rule::Id("UnregisteredRule")).has_value());
}

}  // namespace
}  // namespace rules
}  // namespace api
}  // namespace maliput
