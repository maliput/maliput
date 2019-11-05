#include "maliput/api/rules/discrete_value_rule_state_provider.h"

#include <gtest/gtest.h>

#include "maliput/api/rules/discrete_value_rule.h"
#include "maliput/api/rules/rule.h"
#include "maliput/test_utilities/mock.h"
#include "maliput/test_utilities/rules_compare.h"
#include "maliput/test_utilities/rules_test_utilities.h"

namespace maliput {
namespace api {
namespace rules {
namespace {

// Mock class to evaluate DiscreteValueRuleStateProvider interface.
class MockDiscreteValueRuleStateProvider : public DiscreteValueRuleStateProvider {
 public:
  static const Rule::Id kRuleId;
  static DiscreteValueRule::DiscreteValue MakeCurrentDiscreteValue();
  static DiscreteValueRule::DiscreteValue MakeNextDiscreteValue();

 private:
  drake::optional<StateResult> DoGetState(const Rule::Id& id) const override {
    if (id == kRuleId) {
      return StateResult{MakeCurrentDiscreteValue(),
                         StateResult::Next{MakeNextDiscreteValue(), 123.456 /* duration */}};
    }
    return {};
  }
};

const Rule::Id MockDiscreteValueRuleStateProvider::kRuleId{"RuleId"};

DiscreteValueRule::DiscreteValue MockDiscreteValueRuleStateProvider::MakeCurrentDiscreteValue() {
  return MakeDiscreteValue(Rule::State::kStrict, maliput::api::test::CreateEmptyRelatedRules(),
                           maliput::api::test::CreateEmptyRelatedUniqueIds(), "current_state");
}

DiscreteValueRule::DiscreteValue MockDiscreteValueRuleStateProvider::MakeNextDiscreteValue() {
  return MakeDiscreteValue(Rule::State::kStrict, maliput::api::test::CreateEmptyRelatedRules(),
                           maliput::api::test::CreateEmptyRelatedUniqueIds(), "next_state");
}

GTEST_TEST(DiscreteValueRuleStateProviderTest, ExerciseInterface) {
  const MockDiscreteValueRuleStateProvider dut;
  const drake::optional<DiscreteValueRuleStateProvider::StateResult> result =
      dut.GetState(MockDiscreteValueRuleStateProvider::kRuleId);

  EXPECT_TRUE(result.has_value());
  EXPECT_TRUE(MALIPUT_IS_EQUAL(result->state, MockDiscreteValueRuleStateProvider::MakeCurrentDiscreteValue()));
  EXPECT_TRUE(result->next.has_value());
  EXPECT_TRUE(MALIPUT_IS_EQUAL(result->next->state, MockDiscreteValueRuleStateProvider::MakeNextDiscreteValue()));
  EXPECT_TRUE(result->next->duration_until.has_value());
  EXPECT_EQ(result->next->duration_until.value(), 123.456);

  EXPECT_FALSE(dut.GetState(Rule::Id("UnregisteredRule")).has_value());
}

}  // namespace
}  // namespace rules
}  // namespace api
}  // namespace maliput
