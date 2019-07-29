#include "maliput/api/rules/discrete_value_rule_state_provider.h"

#include <gtest/gtest.h>

namespace maliput {
namespace api {
namespace rules {
namespace {

// Mock class to evaluate DiscreteValueRuleStateProvider interface.
class MockDiscreteValueRuleStateProvider : public DiscreteValueRuleStateProvider {
 public:
  static const Rule::Id kRuleId;

 private:
  drake::optional<DiscreteValueResult> DoGetState(const Rule::Id& id) const override {
    if (id == kRuleId) {
      return DiscreteValueResult{"current_state",
                                 DiscreteValueResult::Next{"next_state" /* state_value */, {123.456} /* duration */}};
    }
    return {};
  }
};

const Rule::Id MockDiscreteValueRuleStateProvider::kRuleId{"RuleId"};

GTEST_TEST(DiscreteValueRuleStateProviderTest, ExerciseInterface) {
  const MockDiscreteValueRuleStateProvider dut;
  const drake::optional<DiscreteValueRuleStateProvider::DiscreteValueResult> result =
      dut.GetState(MockDiscreteValueRuleStateProvider::kRuleId);

  EXPECT_TRUE(result.has_value());
  EXPECT_EQ(result->state_value, "current_state");
  EXPECT_TRUE(result->next.has_value());
  EXPECT_EQ(result->next->state_value, "next_state");
  EXPECT_TRUE(result->next->duration_until.has_value());
  EXPECT_EQ(result->next->duration_until.value(), 123.456);

  EXPECT_FALSE(dut.GetState(Rule::Id("UnregisteredRule")).has_value());
}

}  // namespace
}  // namespace rules
}  // namespace api
}  // namespace maliput
