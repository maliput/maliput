#include "maliput/base/manual_range_value_rule_state_provider.h"

#include <stdexcept>

#include <gtest/gtest.h>

#include "maliput/api/rules/range_value_rule.h"
#include "maliput/api/rules/rule.h"
#include "maliput/common/assertion_error.h"
#include "maliput/test_utilities/mock.h"
#include "maliput/test_utilities/rules_compare.h"
#include "maliput/test_utilities/rules_test_utilities.h"

namespace maliput {
namespace test {
namespace {

using maliput::api::rules::MakeRange;
using maliput::api::rules::RangeValueRule;
using maliput::api::rules::RangeValueRuleStateProvider;
using maliput::api::rules::RoadRulebook;
using maliput::api::rules::Rule;

class ManualRangeValueRuleStateProviderTest : public ::testing::Test {
 protected:
  const Rule::Id kRuleId{"rvrt/rvr_id"};
  const Rule::Id kUnknownRuleId{"rvrt/unknown_id"};
  const RangeValueRule::Range kRangeA{maliput::api::test::CreateRange()};
  const RangeValueRule::Range kInvalidRange{MakeRange(
      Rule::State::kStrict, maliput::api::test::CreateEmptyRelatedRules(), "invalid", 456. /* min */, 789. /* max */)};

  void SetUp() override {
    const maliput::api::test::RoadRulebookBuildFlags kRulebookBuildFlags{
        false /* add_right_of_way */, {} /* right_of_way_build_flags */,   false /* add_direction_usage */,
        false /* add_speed_limit */,  false /* add_discrete_value_rule */, true /* add_range_value_rule */};
    road_rulebook_ = maliput::api::test::CreateRoadRulebook(kRulebookBuildFlags);
  }

  std::unique_ptr<RoadRulebook> road_rulebook_;
};

TEST_F(ManualRangeValueRuleStateProviderTest, ConstructorConstraints) {
  EXPECT_THROW(ManualRangeValueRuleStateProvider(nullptr), maliput::common::assertion_error);
  EXPECT_NO_THROW(ManualRangeValueRuleStateProvider(road_rulebook_.get()));
}

TEST_F(ManualRangeValueRuleStateProviderTest, BasicTest) {
  ManualRangeValueRuleStateProvider dut(road_rulebook_.get());

  // No rule with specified ID exists.
  EXPECT_FALSE(dut.GetState(kRuleId).has_value());
  EXPECT_THROW(dut.SetState(kRuleId, kRangeA, {}), std::out_of_range);

  // Tries to register a state to an invalid Rule::Id.
  EXPECT_THROW(dut.Register(kUnknownRuleId, kRangeA), std::out_of_range);
  // Tries to register an invalid state to a valid Rule.
  EXPECT_THROW(dut.Register(kRuleId, kInvalidRange), maliput::common::assertion_error);
  // Registers a valid state.
  EXPECT_NO_THROW(dut.Register(kRuleId, kRangeA));
  // Tries to register a state to an already registered rule.
  EXPECT_THROW(dut.Register(kRuleId, kRangeA), std::logic_error);

  // Adds a state and evaluates the result.
  drake::optional<RangeValueRuleStateProvider::StateResult> result = dut.GetState(kRuleId);
  EXPECT_TRUE(result.has_value());
  EXPECT_TRUE(MALIPUT_IS_EQUAL(result->range_state, kRangeA));
  EXPECT_EQ(result->next, drake::nullopt);

  // Tries to set the state to an unknown Rule::Id in `rulebook_`.
  EXPECT_THROW(dut.SetState(kUnknownRuleId, kRangeA, {}), std::out_of_range);
  // Tries to set an invalid state to the rule.
  EXPECT_THROW(dut.SetState(kRuleId, kInvalidRange, {}), maliput::common::assertion_error);
  // Tries to set an invalid next state to the rule.
  EXPECT_THROW(dut.SetState(kRuleId, kRangeA, {kInvalidRange}), maliput::common::assertion_error);
  // Sets a valid next state to the rule.
  EXPECT_NO_THROW(dut.SetState(kRuleId, kRangeA, {kRangeA}));

  // Evaluates the result.
  result = dut.GetState(kRuleId);
  EXPECT_TRUE(result.has_value());
  EXPECT_TRUE(MALIPUT_IS_EQUAL(result->range_state, kRangeA));
  EXPECT_TRUE(result->next.has_value());
  EXPECT_TRUE(MALIPUT_IS_EQUAL(result->next->range_state, kRangeA));
  EXPECT_FALSE(result->next->duration_until.has_value());
}

}  // namespace
}  // namespace test
}  // namespace maliput
