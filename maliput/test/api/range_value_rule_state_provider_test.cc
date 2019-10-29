#include "maliput/api/rules/range_value_rule_state_provider.h"

#include <gtest/gtest.h>

#include "maliput/api/rules/range_value_rule.h"
#include "maliput/api/rules/rule.h"
#include "maliput/test_utilities/mock.h"
#include "maliput/test_utilities/rules_compare.h"
#include "maliput/test_utilities/rules_test_utilities.h"

namespace maliput {
namespace api {
namespace rules {
namespace {

// Mock class to evaluate RangeValueRuleStateProvider interface.
class MockRangeValueRuleStateProvider : public RangeValueRuleStateProvider {
 public:
  static const Rule::Id kRuleId;
  static RangeValueRule::Range MakeCurrentRange();
  static RangeValueRule::Range MakeNextRange();

 private:
  drake::optional<StateResult> DoGetState(const Rule::Id& id) const override {
    if (id == kRuleId) {
      return StateResult{MakeCurrentRange(), StateResult::Next{MakeNextRange(), {123.456} /* duration */}};
    }
    return {};
  }
};

const Rule::Id MockRangeValueRuleStateProvider::kRuleId{"RuleId"};

RangeValueRule::Range MockRangeValueRuleStateProvider::MakeCurrentRange() {
  return MakeRange(Rule::State::kStrict, maliput::api::test::CreateEmptyRelatedRules(),
                   maliput::api::test::CreateEmptyRelatedUniqueIds(), "current_range_description", 56. /* min */,
                   78. /* max*/);
}

RangeValueRule::Range MockRangeValueRuleStateProvider::MakeNextRange() {
  return MakeRange(Rule::State::kStrict, maliput::api::test::CreateEmptyRelatedRules(),
                   maliput::api::test::CreateEmptyRelatedUniqueIds(), "next_range_description", 12. /* min */,
                   4. /* max*/);
}

GTEST_TEST(RangeValueRuleStateProviderTest, ExerciseInterface) {
  const MockRangeValueRuleStateProvider dut;
  const drake::optional<RangeValueRuleStateProvider::StateResult> result =
      dut.GetState(MockRangeValueRuleStateProvider::kRuleId);

  EXPECT_TRUE(result.has_value());
  EXPECT_TRUE(MALIPUT_IS_EQUAL(result->state, MockRangeValueRuleStateProvider::MakeCurrentRange()));
  EXPECT_TRUE(result->next.has_value());
  EXPECT_TRUE(MALIPUT_IS_EQUAL(result->next->state, MockRangeValueRuleStateProvider::MakeNextRange()));
  EXPECT_TRUE(result->next->duration_until.has_value());
  EXPECT_EQ(result->next->duration_until.value(), 123.456);

  EXPECT_FALSE(dut.GetState(Rule::Id("UnregisteredRule")).has_value());
}

}  // namespace
}  // namespace rules
}  // namespace api
}  // namespace maliput
